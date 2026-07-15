; CONF_TNY.A51 - RTX51 Tiny configuration
;
; 关键配置:
;   INT_REGBANK = 1  (RTX51 使用寄存器组1, 用户 ISR 用组0)
;   TIMESHARING = 0  (关闭时间片轮转, 纯协作式)
;   RAMTOP      = 0  (自动检测)
;
; 注意: RTX51 Tiny 占用 Timer0, TimeStamp_Init() 必须改用其他定时器

$DEBUG

INT_REGBANK    EQU     1       ; RTX51 用寄存器组1, 用户ISR用组0

; 任务配置: 最多3个任务 (0=Init, 1=Modbus, 2=Measure)
; 每个任务需要约10字节 xdata + 栈空间
RAMTOP         EQU     0FFH    ; 栈顶 (根据实际 RAM 调整, 0xFF=自动)

; 时间片 = 0: 纯协作式 (任务必须主动 os_wait 让出 CPU)
TIMESHARING    EQU     0

; 系统时钟: 10ms tick (100Hz)
; Timer0 24MHz/12T=2MHz: 10ms = 20000 machine cycles
ENVTICK        EQU     1
ENVCLOCK       EQU     1
INTCLOCK       EQU     20000  ; 10ms tick

        END
