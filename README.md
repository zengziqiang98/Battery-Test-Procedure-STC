# 电池测试程序 — 项目整合

## 项目概览

本项目整合了电池充放电测试治具的全部核心代码，包含三个子系统：

| 子系统 | 平台 | 语言 | 功能 |
|--------|------|------|------|
| ESP32 固件 | PlatformIO (Arduino) | C++ | 充放电控制、I2C 通信、数据采集 |
| STC8H 固件 | Keil C51 | C | ADC 采集、PWM 输出、I2C 从机 |
| Python UI | Windows 桌面 | Python (tkinter) | 上位机图形界面、Modbus 通信、数据导出 |

## 目录结构

+"`"+
codex-电池测试程序/
├── esp32/                  # ESP32 PlatformIO 工程
│   ├── src/
│   │   ├── main.cpp        # 主程序
│   │   └── BatteryTester.cpp # 充放电控制逻辑
│   ├── include/
│   │   └── BatteryTester.h # 头文件
│   ├── platformio.ini      # PlatformIO 配置
│   └── .gitignore
├── stc8h/                  # STC8H Keil C51 工程
│   ├── src/                # 全部源码 (.c/.h)
│   │   ├── main.c          # 主程序
│   │   ├── i2c_cmd.h       # I2C 命令定义
│   │   ├── ADC.c, capacity.c, ...
│   │   └── STC8G_H_*.c/h   # 外设库文件
│   ├── 电池STC8程序.uvproj # Keil 工程文件
│   ├── 电池STC8程序.uvopt
│   ├── .clang-format
│   └── .gitignore
├── python/                 # Python 上位机
│   ├── 测试.py             # 主 UI 程序
│   ├── modbus_thread.py    # Modbus RTU 通信线程
│   ├── protocol_thread.py  # 协议解析线程
│   ├── modbus_scan.py      # Modbus 设备扫描工具
│   ├── gen_main.py         # 代码生成脚本
│   ├── battery_data.json   # 电池参数配置
│   ├── battery_tester_config.json
│   ├── BatteryTester.spec  # PyInstaller 打包配置
│   └── assets/             # 图标资源
├── protocol/               # 跨平台共享协议文件
│   ├── protocol.c / .h     # 串口通信协议
│   ├── modbus_slave.c / .h # Modbus 从站实现
│   ├── 协议定义.h          # 中文协议定义
│   └── main.c / main_mcu.c / modbus_main.c / comm_mcu.c / uart.c
├── data/                   # 测试数据与导出
│   ├── 测试数据/           # 25 组 autosave CSV 数据
│   └── 电池测试_*.xlsx     # Excel 导出示例
├── docs/
│   └── A0电池测试治具技术要求2025-12-29.docx
├── history/                # 关键历史版本备份
│   ├── esp32/              # ESP32 早期代码 + Python 里程碑备份
│   └── stc8h/              # STC8H Modbus/I2C 关键里程碑备份 + 编译日志
└── ide/                    # IDE 配置文件备份
    ├── esp32/.vscode/
    └── stc8h/.vscode/, .eide/
+"`"+

## 快速开始

### ESP32 固件编译
+"`"+ash
cd esp32
pio run
+"`"+

### STC8H 固件编译
用 Keil C51 打开 +"stc8h/电池STC8程序.uvproj"+

### Python UI 运行
+"`"+ash
cd python
python 测试.py
+"`"+

## 备注

- 原有项目目录保持不变，本次为**复制移植**，未删除任何原始文件
- Build 产物（.pio/, build/, Objects/, Listings/, __pycache__/）未移植，可随时从源码重新编译生成
- 大量 .bak 历史备份仅保留了关键里程碑版本，完整备份仍在原目录中
