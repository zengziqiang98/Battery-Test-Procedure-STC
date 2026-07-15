# -*- coding: utf-8 -*-
# modbus_thread.py - Modbus RTU通信线程 (替代原SerialThread)
#
# 用法: 在 MainWindow 中替代 SerialThread 的创建和信号连接
#
# from modbus_thread import ModbusThread
# self.serial_thread = ModbusThread(port, baudrate=115200)
# self.serial_thread.data_received.connect(self.on_data_received)
# self.serial_thread.log_message.connect(self.on_log_message)
# self.serial_thread.error_occurred.connect(self.on_serial_error)
# self.serial_thread.start()
#
# 寄存器映射与 STC8H modbus_slave.h 一致
#
# 注意: 此文件需要 pymodbus
#   pip install pymodbus

import threading
import time
import struct
from PyQt5.QtCore import QThread, pyqtSignal

try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.exceptions import ModbusException
except ImportError:
    ModbusSerialClient = None
    ModbusException = Exception


# ========== Modbus寄存器地址 (与STC8H固件 modbus_slave.h 一致) ==========
REG_CMD               = 0x0000
REG_STATUS_CHARGE     = 0x0001
REG_STATUS_DISCHARGE  = 0x0002
REG_CHARGE_MAH        = 0x0003
REG_DISCHARGE_MAH     = 0x0004
REG_BAT_VOLT          = 0x0005
REG_CHARGE_CURRENT    = 0x0006
REG_DISCHARGE_CURRENT = 0x0007
REG_OVERC_PROT        = 0x0008
REG_OVERD_PROT        = 0x0009
REG_CYCLE_NUM         = 0x000A
REG_DEVICE_ID         = 0x000B
REG_SET_OVERC_PROT    = 0x0010
REG_SET_OVERD_PROT    = 0x0011
REG_SET_CYCLE_NUM     = 0x0012
REG_SET_CHG_VOLT      = 0x0013
REG_SET_CHG_CURRENT   = 0x0014
REG_SET_BAT_TYPE      = 0x0015
REG_CYCLE_DISCH_BASE  = 0x0020
REG_CYCLE_CHG_BASE    = 0x0030

# ========== Modbus命令码 (写入CMD寄存器) ==========
CMD_START_DISCHARGE  = 0x0001
CMD_STOP_DISCHARGE   = 0x0002
CMD_START_CHARGE     = 0x0003
CMD_STOP_CHARGE      = 0x0004
CMD_EMERGENCY_STOP   = 0x0005
CMD_BUZZER_ON        = 0x0006
CMD_BUZZER_OFF       = 0x0007

# ========== 状态位 ==========
STAT_CHARGING           = 0x01
STAT_CHARGE_COMPLETE    = 0x02
STAT_OVERC_ALARM        = 0x04
STAT_NO_BATTERY         = 0x08
STAT_CHARGE_TIMEOUT     = 0x10
STAT_DISCHARGING        = 0x01
STAT_DISCHARGE_COMPLETE = 0x02
STAT_OVERD_ALARM        = 0x04
STAT_DISCHARGE_TIMEOUT  = 0x10

# ========== 模块地址范围 ==========
MODULE_ADDR_MIN = 1
MODULE_ADDR_MAX = 64
MODULE_COUNT = 64

# ========== 批量读取参数 ==========
BATCH_READ_START = REG_STATUS_CHARGE   # 0x0001
BATCH_READ_COUNT = 11                  # 读 0x0001 ~ 0x000B

# ========== 离线模块跳过阈值 ==========
OFFLINE_SKIP_THRESHOLD = 5    # 连续N次无响应后降低轮询频率
OFFLINE_SKIP_INTERVAL  = 8    # 离线模块每N轮才查询一次


class ModbusThread(QThread):
    """Modbus RTU通信线程, 替代原SerialThread"""

    # ---- 旧协议命令码映射 ----
    # 单模块命令 (0x01-0x0C) -> Modbus寄存器地址或命令码
    _CMD_REG_MAP = {
        # 旧协议码 -> Modbus命令寄存器内容
        0x01: CMD_START_DISCHARGE,   # CMD_START_DISCHARGE
        0x02: CMD_STOP_DISCHARGE,    # CMD_STOP_DISCHARGE
        0x03: CMD_START_CHARGE,      # CMD_START_CHARGE
        0x04: CMD_STOP_CHARGE,       # CMD_STOP_CHARGE
        # 旧协议码 -> Modbus配置寄存器地址
        0x05: REG_SET_OVERC_PROT,    # CMD_SET_OVERC_PROT
        0x06: REG_SET_OVERD_PROT,    # CMD_SET_OVERD_PROT
        0x07: REG_SET_CYCLE_NUM,     # CMD_SET_CYCLE_NUM
        0x08: REG_SET_CHG_VOLT,      # CMD_SET_CHARGE_VOLT
        0x09: REG_SET_CHG_CURRENT,   # CMD_SET_CHARGE_CUR
        0x0A: REG_SET_BAT_TYPE,      # CMD_SET_BAT_TYPE
        0x0B: CMD_BUZZER_ON,         # CMD_BUZZER_ON
        0x0C: CMD_BUZZER_OFF,        # CMD_BUZZER_OFF
    }

    # 广播命令 (0x20-0x28) -> Modbus命令码/寄存器
    _BCAST_MAP = {
        0x20: CMD_START_CHARGE,      # CMD_ALL_START_CHARGE
        0x21: CMD_STOP_CHARGE,       # CMD_ALL_STOP_CHARGE
        0x22: CMD_START_DISCHARGE,   # CMD_ALL_START_DISCHARGE
        0x23: CMD_STOP_DISCHARGE,    # CMD_ALL_STOP_DISCHARGE
        0x24: REG_SET_CYCLE_NUM,     # CMD_ALL_SET_CYCLE_NUM
        0x25: REG_SET_BAT_TYPE,      # CMD_ALL_SET_BAT_TYPE
        0x26: CMD_EMERGENCY_STOP,    # CMD_ALL_EMERGENCY_STOP
        0x27: CMD_BUZZER_ON,         # CMD_ALL_BUZZER_ON
        0x28: CMD_BUZZER_OFF,        # CMD_ALL_BUZZER_OFF
    }

    # ---- PyQt5 信号 ----
    data_received = pyqtSignal(list)       # list[dict|None] 64个模块数据
    log_message = pyqtSignal(str, str)     # msg, category: comm|error|info
    error_occurred = pyqtSignal(str)       # error msg
    command_response = pyqtSignal(int, int, bytes)  # cmd, status, data
    first_scan_done = pyqtSignal()         # 首扫完成

    def __init__(self, port, baudrate=115200, timeout=0.10):
        """
        @param port      串口端口名 (e.g. 'COM3')
        @param baudrate  波特率 (默认115200, 必须与STC8H固件一致)
        @param timeout   单次Modbus读写超时 (秒, 默认30ms)
        """
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.client = None
        self.running = False
        self.lock = threading.Lock()

        self._poll_round = 0
        self._active_set = set()
        self._offline_count = {}

    def run(self):
        if ModbusSerialClient is None:
            self.error_occurred.emit("pymodbus 未安装, 请执行: pip install pymodbus")
            return

        try:
            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity='N',
                stopbits=1,
                bytesize=8
            )
            if not self.client.connect():
                self.error_occurred.emit(f"Modbus连接失败: {self.port}")
                return

            self.log_message.emit(f"Modbus RTU 已连接 {self.port} @ {self.baudrate}", "comm")
            self.running = True

            while self.running:
                modules_data = [None] * MODULE_COUNT  # 始终 64 元素
                self._poll_round += 1

                # 有已知活跃模块就只查活跃的, 否则全扫; 每 30 轮强制全扫
                if self._active_set and (self._poll_round % 30 != 0):
                    scan_addrs = list(self._active_set)  # 副本, 避免遍历时修改
                else:
                    scan_addrs = range(MODULE_ADDR_MIN, MODULE_ADDR_MAX + 1)

                for addr in scan_addrs:
                    try:
                        rr = self.client.read_holding_registers(
                            address=BATCH_READ_START,
                            count=BATCH_READ_COUNT,
                            slave=addr
                        )
                        if rr.isError():
                            cnt = self._offline_count.get(addr, 0) + 1
                            self._offline_count[addr] = cnt
                            if cnt >= 5:
                                self._active_set.discard(addr)
                            continue

                        self._offline_count[addr] = 0
                        self._active_set.add(addr)
                        regs = rr.registers
                        modules_data[addr - 1] = {
                            'addr': addr,
                            'status_charge': regs[0],
                            'status_discharge': regs[1],
                            'charge_mah': regs[2],
                            'discharge_mah': regs[3],
                            'voltage': regs[4],
                            'charge_current': regs[5],
                            'discharge_current': regs[6],
                            'overc_prot': regs[7],
                            'overd_prot': regs[8],
                            'cycle': regs[9],
                            'device_id': regs[10],
                            'status': self._synthesize_status(regs[0], regs[1]),
                            'ck_fail_count': 0,
                        }
                    except ModbusException:
                        cnt = self._offline_count.get(addr, 0) + 1
                        self._offline_count[addr] = cnt
                        if cnt >= 5:
                            self._active_set.discard(addr)
                    except Exception:
                        pass

                # 首扫完成后发信号
                if self._poll_round == 1:
                    online_cnt = sum(1 for m in modules_data if m is not None)
                    self.log_message.emit(f"首扫完成: {online_cnt}个在线", "comm")
                    self.first_scan_done.emit()

                self.data_received.emit(modules_data)
                time.sleep(0.02)

        except Exception as e:
            self.error_occurred.emit(f"Modbus线程异常: {e}")
        finally:
            if self.client:
                self.client.close()

    def _synthesize_status(self, status_chg, status_dchg):
        """合成兼容旧协议的 status 字节"""
        stat = 0
        if status_chg & STAT_CHARGING:
            stat |= 0x01          # bit0: 充电中
        if status_dchg & STAT_DISCHARGING:
            stat |= 0x02          # bit1: 放电中
        if status_chg & STAT_OVERC_ALARM:
            stat |= 0x04          # bit2: 过充报警
        if status_dchg & STAT_OVERD_ALARM:
            stat |= 0x08          # bit3: 过放报警
        if status_chg & STAT_CHARGE_COMPLETE:
            stat |= 0x10          # bit4: 充电完成
        if status_dchg & STAT_DISCHARGE_COMPLETE:
            stat |= 0x20          # bit5: 放电完成
        if status_chg & STAT_NO_BATTERY:
            stat |= 0x40          # bit6: 无电池
        return stat

    # ========== 命令发送 ==========

    def send_command(self, cmd_code, addr, data=None):
        """
        发送Modbus写命令到指定模块 (非阻塞, 自动映射旧协议码)

        @param cmd_code  旧协议命令码 (0x01-0x0C) 或 Modbus寄存器地址
        @param addr      模块地址 (1-64)
        @param data      配置数据 (list[int], 如 [high_byte, low_byte])
        @return True/False
        """
        if self.client is None:
            return False

        # 查找映射
        mapped = self._CMD_REG_MAP.get(cmd_code, cmd_code)

        if mapped >= 0x0010:
            # 配置寄存器 (REG_SET_OVERC_PROT ~ REG_SET_BAT_TYPE)
            reg_addr = mapped
            reg_val = data[0] if data else 0
        elif mapped <= 0x0007:
            # 命令码 (0x0001 ~ 0x0007): 写入 CMD 寄存器
            reg_addr = REG_CMD
            reg_val = mapped
        else:
            # 未知, 尝试作为寄存器地址
            reg_addr = mapped
            reg_val = data[0] if data else 0

        try:
            self.client.write_register(
                address=reg_addr,
                value=reg_val,
                slave=addr
            )
            return True
        except Exception as e:
            self.log_message.emit(f"send_cmd addr={addr} cmd={cmd_code:#04x}: {e}", "error")
            return False

    def send_broadcast(self, cmd_code):
        """
        广播命令到所有模块 (Modbus 地址 0)

        @param cmd_code  Modbus命令码 (0x0001-0x0007)
        @return True/False
        """
        if self.client is None:
            return False
        try:
            self.client.write_register(
                address=REG_CMD,
                value=cmd_code,
                slave=0  # Modbus 广播地址
            )
            self.log_message.emit(f"Broadcast cmd=0x{cmd_code:04X}", "comm")
            return True
        except Exception as e:
            self.log_message.emit(f"Broadcast error: {e}", "error")
            return False

    def send_all_command(self, cmd_code, data=None):
        """
        兼容旧接口: 自动映射旧协议广播码→Modbus码并广播

        广播命令 (0x20-0x28) 自动映射为 Modbus 命令码写入地址0
        配置类广播 (0x24/0x25) 需要带 data 参数, 逐个写入在线模块
        """
        mapped = self._BCAST_MAP.get(cmd_code, cmd_code)

        if cmd_code in (0x24, 0x25) and data is not None:
            # 配置类广播: 写配置寄存器 (不是CMD寄存器), 逐个在线模块写入
            reg_addr = mapped
            reg_val = data[0] if data else 0
            for addr in range(MODULE_ADDR_MIN, MODULE_ADDR_MAX + 1):
                try:
                    self.client.write_register(address=reg_addr, value=reg_val, slave=addr)
                except Exception:
                    pass  # 忽略离线模块
            return True
        else:
            # 命令类广播: 写CMD寄存器到地址0
            return self.send_broadcast(mapped)

    # ========== 兼容旧 SerialThread API ==========

    @property
    def ser(self):
        """兼容旧代码的 ser.is_open 检查"""
        class _Ser:
            is_open = False
        if self.client is not None:
            try:
                _Ser.is_open = self.client.is_socket_open()
            except Exception:
                _Ser.is_open = True
        return _Ser()

    def send_batch_read(self):
        """兼容旧接口: ModbusThread 自动轮询, 手动触发无需操作"""
        pass

    def send_command_sync(self, cmd_code, addr, data=None, timeout_ms=300):
        """同步发送命令 (兼容旧接口)"""
        ok = self.send_command(cmd_code, addr, data)
        if timeout_ms:
            time.sleep(timeout_ms / 1000.0)
        return ok

    def stop(self):
        """停止线程"""
        self.running = False
        if self.client:
            self.client.close()
        self.wait(1000)
