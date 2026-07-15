# -*- coding: utf-8 -*-
"""
Modbus RTU 模块扫描测试工具
用法: python modbus_scan.py COM3 115200
"""

import sys
import time
from pymodbus.client import ModbusSerialClient

def scan_modules(port, baudrate=115200):
    """扫描 Modbus 总线上的所有模块"""
    client = ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        timeout=0.1,
        parity='N',
        stopbits=1,
        bytesize=8
    )

    print(f"连接到 {port} @ {baudrate} baud...")
    if not client.connect():
        print(f"ERROR: 无法打开串口 {port}")
        return

    print("连接成功，开始扫描模块...")
    print("-" * 60)

    found = 0
    for addr in range(1, 65):
        try:
            rr = client.read_holding_registers(
                address=0x0001,
                count=11,
                slave=addr
            )
            if rr.isError():
                sys.stdout.write(f"[{addr:3d}] 无响应\r")
                sys.stdout.flush()
            else:
                regs = rr.registers
                found += 1
                print(f"\n[{addr:3d}] *** 在线 ***")
                print(f"    充电状态:     0x{regs[0]:04X}")
                print(f"    放电状态:     0x{regs[1]:04X}")
                print(f"    充电mAh:      {regs[2]} mAh")
                print(f"    放电mAh:      {regs[3]} mAh")
                print(f"    电池电压:     {regs[4]} mV")
                print(f"    充电电流:     {regs[5]} mA")
                print(f"    放电电流:     {regs[6]} mA")
                print(f"    过充保护值:   {regs[7]} mV")
                print(f"    过放保护值:   {regs[8]} mV")
                print(f"    当前循环:     {regs[9]}")
                print(f"    设备ID:       0x{regs[10]:04X}")
        except Exception as e:
            print(f"\n[{addr:3d}] 错误: {e}")

    print("\n" + "=" * 60)
    print(f"扫描完成: {found}/64 个模块在线")

    client.close()


def read_module_detail(port, baudrate, addr):
    """读取指定模块的完整信息，包括循环历史"""
    client = ModbusSerialClient(
        port=port, baudrate=baudrate, timeout=0.1,
        parity='N', stopbits=1, bytesize=8
    )
    if not client.connect():
        print(f"ERROR: 无法打开串口 {port}")
        return

    print(f"\n=== 模块 {addr} 详细信息 ===")
    # 批量读主数据
    rr = client.read_holding_registers(address=0x0001, count=11, slave=addr)
    if rr.isError():
        print("无响应!")
        client.close()
        return
    regs = rr.registers
    print(f"电压: {regs[4]}mV | 充电: {regs[2]}mAh/{regs[5]}mA | 放电: {regs[3]}mAh/{regs[6]}mA")
    print(f"状态字节: chg=0x{regs[0]:02X} dchg=0x{regs[1]:02X} | 循环: {regs[9]} | ID: 0x{regs[10]:04X}")

    # 读循环放电历史
    rr2 = client.read_holding_registers(address=0x0020, count=10, slave=addr)
    if not rr2.isError():
        dchg_hist = [str(v) for v in rr2.registers]
        print(f"放电历史(mAh): {', '.join(dchg_hist)}")

    # 读循环充电历史
    rr3 = client.read_holding_registers(address=0x0030, count=10, slave=addr)
    if not rr3.isError():
        chg_hist = [str(v) for v in rr3.registers]
        print(f"充电历史(mAh): {', '.join(chg_hist)}")

    client.close()


def test_write(port, baudrate, addr):
    """测试写命令 - 蜂鸣器 (仅验证通信)"""
    client = ModbusSerialClient(
        port=port, baudrate=baudrate, timeout=0.1,
        parity='N', stopbits=1, bytesize=8
    )
    if not client.connect():
        print(f"ERROR: 无法打开串口 {port}")
        return

    print(f"\n=== 向模块 {addr} 发送测试命令 ===")
    # 读设备ID确认通信
    rr = client.read_holding_registers(address=0x000B, count=1, slave=addr)
    if rr.isError():
        print(f"模块 {addr} 无响应，无法测试")
    else:
        print(f"设备ID: 0x{rr.registers[0]:04X}")
        print("通信正常!")

    client.close()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("=" * 60)
    print("  Modbus RTU 模块扫描工具")
    print(f"  端口: {port}  波特率: {baud}")
    print("=" * 60)

    # 检查一下是否需要读单个模块
    if len(sys.argv) > 3 and sys.argv[3] == "detail":
        addr = int(sys.argv[4]) if len(sys.argv) > 4 else 1
        read_module_detail(port, baud, addr)
    elif len(sys.argv) > 3 and sys.argv[3] == "test":
        addr = int(sys.argv[4]) if len(sys.argv) > 4 else 1
        test_write(port, baud, addr)
    else:
        scan_modules(port, baud)
