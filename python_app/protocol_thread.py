# -*- coding: utf-8 -*-
"""自定义协议通信线程 v4 — 命令队列, 轮询不阻塞"""

import threading, time, struct, serial
from collections import deque
from PyQt5.QtCore import QThread, pyqtSignal

CMD_BATCH_READ = 0x08; CMD_WRITE = 0x02; RESP_OK = 0x00
_OLD_CMD_MAP = {0x01:0x05,0x02:0x06,0x03:0x03,0x04:0x04,
    0x05:('w',0x10,2),0x06:('w',0x11,2),0x07:('w',0x12,1),
    0x08:('w',0x13,1),0x09:('w',0x14,1),0x0A:('w',0x15,1)}
_BCAST_MAP = {0x20:0x03,0x21:0x04,0x22:0x05,0x23:0x06,
    0x24:('w',0x12,1),0x25:('w',0x15,1),0x26:0x07}
MODULE_COUNT = 64
POLL_READ_TIMEOUT_SEC = 0.004  # cached bridge batch read is ~2.3ms; keep headroom
PROBE_TIMEOUT_SEC = 0.0020     # 1.8ms can miss during full scans; 2ms is stable
MIN_ROUND_PERIOD_SEC = 0.0     # no forced 1s sleep; fastest polling
EXECUTOR_OFFLINE = object()

def xor_checksum(data):
    c = 0
    for b in data: c ^= b
    return c

class ProtocolThread(QThread):
    data_received = pyqtSignal(list)
    log_message = pyqtSignal(str, str)
    error_occurred = pyqtSignal(str)
    command_response = pyqtSignal(int, int, bytes)
    first_scan_done = pyqtSignal()

    def __init__(self, port, baudrate=256000):
        super().__init__()
        self.port, self.baudrate = port, baudrate
        self._s = None; self.running = False
        self._cmd_queue = deque()  # 命令队列: (func, args, kwargs)
        self._cmd_lock = threading.Lock()
        self._round = 0; self._active = set(); self._offline = {}

    def run(self):
        try:
            self._s = serial.Serial(self.port, self.baudrate, timeout=0)
            self._s.dtr = False
            self._s.rts = False
            self.log_message.emit(f"协议 {self.port}@{self.baudrate}", "comm")
            self.running = True

            while self.running:
                # 1. 处理所有待发命令
                self._drain_commands()

                # 2. 扫描一批
                data = [None] * MODULE_COUNT
                self._round += 1

                do_full = (self._round % 1000 == 0)
                scan_all = do_full or not self._active
                if scan_all:
                    addrs = range(1, MODULE_COUNT + 1)
                else:
                    addrs = list(self._active)

                t_scan_start = time.time()
                online_this = []; bridge_only_this = []; offline_this = []

                for a in addrs:
                    bridge_seen = False
                    if scan_all:
                        bridge_seen = self._probe_addr(a)
                    else:
                        bridge_seen = True
                    if not bridge_seen:
                        self._offline[a] = self._offline.get(a, 0) + 1
                        if self._offline[a] >= 30: self._active.discard(a)
                        offline_this.append(str(a))
                        continue
                    regs = self._read_raw(a)
                    if regs is None and not scan_all:
                        bridge_seen = self._probe_addr(a)
                    if regs is None and bridge_seen:
                        self._offline[a] = 0; self._active.add(a)
                        bridge_only_this.append(str(a))
                        data[a - 1] = self._bridge_only_record(a)
                        continue
                    if regs is None:
                        self._offline[a] = self._offline.get(a, 0) + 1
                        if self._offline[a] >= 30: self._active.discard(a)
                        offline_this.append(str(a))
                        continue
                    if regs is EXECUTOR_OFFLINE:
                        self._offline[a] = 0; self._active.add(a)
                        bridge_only_this.append(str(a))
                        data[a - 1] = self._bridge_only_record(a)
                        continue
                    self._offline[a] = 0; self._active.add(a)
                    online_this.append(str(a))
                    sc, sd = regs[0], regs[1]; st = 0
                    if sc & 0x04: st |= 0x01
                    if sd & 0x04: st |= 0x02
                    if sc & 0x01: st |= 0x04
                    if sd & 0x01: st |= 0x08
                    if sc & 0x02: st |= 0x10
                    if sd & 0x02: st |= 0x20
                    if sc & 0x08: st |= 0x40
                    data[a - 1] = {'addr': a, 'voltage': regs[4],
                        'charge_current': regs[5], 'discharge_current': regs[6],
                        'charge_mah': regs[2], 'discharge_mah': regs[3],
                        'status': st, 'cycle': regs[9], 'ck_fail_count': 0,
                        'bridge_online': True, 'executor_online': True}

                self.data_received.emit(data)
                t_scan = (time.time() - t_scan_start)

                if MIN_ROUND_PERIOD_SEC and t_scan < MIN_ROUND_PERIOD_SEC:
                    time.sleep(MIN_ROUND_PERIOD_SEC - t_scan)

                self.log_message.emit(
                    f"轮{self._round} {t_scan*1000:.0f}ms "
                    f"执行在线{len(online_this)} 中转在线执行离线{len(bridge_only_this)} 离线{len(offline_this)}",
                    "comm")

                if self._round == 1:
                    self.first_scan_done.emit()

        except serial.SerialException as e:
            self.error_occurred.emit(f"串口: {e}")
        except Exception as e:
            self.error_occurred.emit(f"线程: {e}")
        finally:
            if self._s:
                try: self._s.close()
                except: pass

    def _drain_commands(self):
        """发送队列中所有待发命令"""
        while True:
            with self._cmd_lock:
                if not self._cmd_queue:
                    return
                item = self._cmd_queue.popleft()
            func, args, kwargs, done, result = item
            try:
                status = func(*args, **kwargs)
            except Exception:
                status = -1
            if result is not None:
                result["status"] = status
            if done is not None:
                done.set()

    def _read_reply_nonblocking(self, expected_len, timeout_sec):
        resp = bytearray()
        deadline = time.perf_counter() + timeout_sec
        while len(resp) < expected_len and time.perf_counter() < deadline:
            waiting = self._s.in_waiting
            if waiting:
                resp.extend(self._s.read(min(expected_len - len(resp), waiting)))
            else:
                time.sleep(0)
        return resp

    def _probe_addr(self, addr):
        """Fast no-IIC probe: read DEVICE_ID register only."""
        if self._s is None: return False
        try:
            frame = bytearray([addr, 0x01, 3, 0x00, 0x0B, 0x01])
            frame.append(xor_checksum(frame))
            self._s.timeout = 0
            self._s.reset_input_buffer()
            self._s.write(frame); self._s.flush()
            resp = self._read_reply_nonblocking(7, PROBE_TIMEOUT_SEC)
            if len(resp) != 7 or resp[0] != addr or resp[1] != 0x81: return False
            if resp[2] != RESP_OK or resp[3] != 2: return False
            if xor_checksum(resp[:-1]) != resp[-1]: return False
            return True
        except Exception:
            return False
    def _read_raw(self, addr):
        """Read one module with a 10ms non-blocking deadline."""
        if self._s is None: return None
        try:
            frame = bytearray([addr, CMD_BATCH_READ, 3, 0x00, 0x01, 0x0B])
            frame.append(xor_checksum(frame))
            self._s.timeout = 0
            self._s.reset_input_buffer()
            self._s.write(frame); self._s.flush()

            resp = self._read_reply_nonblocking(27, POLL_READ_TIMEOUT_SEC)

            if len(resp) < 27: return None
            if resp[0] != addr or resp[1] != (CMD_BATCH_READ | 0x80): return None
            if xor_checksum(resp[:-1]) != resp[-1]: return None
            if resp[3] == 22 and resp[2] != RESP_OK: return EXECUTOR_OFFLINE
            if resp[2] != RESP_OK or resp[3] != 22: return None
            return struct.unpack('>11H', resp[4:26])
        except Exception:
            return None

    def _bridge_only_record(self, addr):
        return {'addr': addr, 'voltage': 0,
            'charge_current': 0, 'discharge_current': 0,
            'charge_mah': 0, 'discharge_mah': 0,
            'status': 0, 'cycle': 0, 'ck_fail_count': 0,
            'bridge_online': True, 'executor_online': False}
    # ===== 命令接口 (入队, 不阻塞轮询) =====

    def send_command(self, cmd_code, addr, data=None):
        m = _OLD_CMD_MAP.get(cmd_code, cmd_code)
        if isinstance(m, tuple):
            _, ra, bc = m
            v = data[0] if data and bc == 1 else ((data[0] << 8) | data[1]) if data and bc == 2 else 0
            with self._cmd_lock:
                self._cmd_queue.append((self._write_raw, (addr, ra, v), {}, None, None))
        else:
            with self._cmd_lock:
                self._cmd_queue.append((self._cmd_raw, (addr, m), {}, None, None))

    def send_all_command(self, cmd_code, data=None):
        m = _BCAST_MAP.get(cmd_code, cmd_code)
        if isinstance(m, tuple):
            _, ra, bc = m
            v = data[0] if data and bc == 1 else 0
            with self._cmd_lock:
                self._cmd_queue.append((self._write_raw, (0, ra, v), {}, None, None))
        else:
            with self._cmd_lock:
                self._cmd_queue.append((self._cmd_raw, (0, m), {}, None, None))

    def send_command_sync(self, cmd_code, addr, data=None, timeout_ms=300):
        """Queue command and wait for the bridge status. Returns 0=OK, 0xFF=fail, -1=timeout/unavailable."""
        if not self.running or self._s is None or not self._s.is_open:
            return -1
        m = _OLD_CMD_MAP.get(cmd_code, cmd_code)
        done = threading.Event()
        result = {"status": -1}
        if isinstance(m, tuple):
            _, ra, bc = m
            v = data[0] if data and bc == 1 else ((data[0] << 8) | data[1]) if data and bc == 2 else 0
            item = (self._write_raw, (addr, ra, v), {}, done, result)
        else:
            item = (self._cmd_raw, (addr, m), {}, done, result)
        with self._cmd_lock:
            self._cmd_queue.append(item)
        if not done.wait(timeout_ms / 1000.0):
            return -1
        return result["status"]

    def _cmd_raw(self, addr, cmd):
        f = bytearray([addr, cmd, 0]); f.append(xor_checksum(f))
        self._s.timeout = 0.03
        self._s.write(f); self._s.flush()
        if addr == 0:
            return 0
        resp = self._s.read(5)
        if len(resp) != 5 or resp[0] != addr or resp[1] != (cmd | 0x80):
            return -1
        if xor_checksum(resp[:-1]) != resp[-1]:
            return -1
        self.command_response.emit(cmd, resp[2], b"")
        return resp[2]

    def _write_raw(self, addr, reg, val):
        d = bytes([(reg>>8)&0xFF, reg&0xFF, (val>>8)&0xFF, val&0xFF])
        f = bytearray([addr, CMD_WRITE, len(d)]); f.extend(d); f.append(xor_checksum(f))
        self._s.timeout = 0.03
        self._s.write(f); self._s.flush()
        if addr == 0:
            return 0
        resp = self._s.read(5)
        if len(resp) != 5 or resp[0] != addr or resp[1] != (CMD_WRITE | 0x80):
            return -1
        if xor_checksum(resp[:-1]) != resp[-1]:
            return -1
        self.command_response.emit(CMD_WRITE, resp[2], b"")
        return resp[2]

    send_batch_read = lambda self: None

    @property
    def ser(self):
        class S: is_open = False
        if self._s: S.is_open = self._s.is_open
        return S()

    def stop(self):
        self.running = False
        if self._s:
            try: self._s.close()
            except: pass
        self.wait(1000)

