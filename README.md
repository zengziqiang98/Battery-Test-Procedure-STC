# Battery Test Procedure STC

电池测试治具项目，当前主线只保留三块正式程序：

| 目录 | 内容 | 说明 |
| --- | --- | --- |
| `python_app/` | Python 上位机 | PyQt5 桌面程序，串口连接中转模块 |
| `firmware/stc8h/bridge/` | STC 中转模块源码 | PC 串口协议 + IIC 主机 + 后台缓存 |
| `firmware/stc8h/src/` | STC 执行模块源码 | ADC、容量计算、充放电控制、IIC 从机 |

## 目录结构

```text
Battery-Test-Procedure-STC/
├── python_app/
│   ├── 测试.py
│   ├── protocol_thread.py
│   ├── battery_tester_config.json
│   └── assets/
├── firmware/
│   └── stc8h/
│       ├── bridge/
│       │   ├── main.c
│       │   ├── protocol.c
│       │   └── protocol.h
│       ├── src/
│       │   ├── main.c
│       │   ├── i2c_cmd.h
│       │   └── STC8G_H_*.c/h
│       ├── bridge_stc8h.uvproj
│       └── executor_stc8h.uvproj
└── docs/
    └── 协议链路对照表.md
```

## 运行上位机

```powershell
cd D:\codex-电池测试程序\python_app
python .\测试.py
```

当前开发环境使用 Python 3.8。串口协议默认波特率为 `256000`，实测中转模块地址为 `0x1D`。

## 编译 STC 固件

中转模块：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'D:\codex-电池测试程序\firmware\stc8h\bridge_stc8h.uvproj' -t 'Bridge_STC8H' -o 'D:\codex-电池测试程序\firmware\stc8h\build_bridge_single.log'
```

执行模块：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'D:\codex-电池测试程序\firmware\stc8h\executor_stc8h.uvproj' -t 'Executor_STC8H' -o 'D:\codex-电池测试程序\firmware\stc8h\build_executor_single.log'
```

生成的 `.hex`、`Objects/`、`Listings/` 和日志文件不纳入 Git。

## 当前协议

```text
Python 上位机 -> 串口协议 -> STC 中转模块 -> IIC -> STC 执行模块
```

中转模块对上位机高频读取的 `0x0001..0x000B` 寄存器使用后台 IIC 缓存，串口批量读不再同步等待 IIC。详细协议见 [docs/协议链路对照表.md](docs/协议链路对照表.md)。
