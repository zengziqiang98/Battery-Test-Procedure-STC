// i2c_cmd.h - I2C协议命令与共享变量定义
#ifndef __I2C_CMD_H
#define __I2C_CMD_H

/*
 * alarmOvercharge 位定义 (unsigned char)
 *   bit0 : 过充报警 (电压超过保护值确认)
 *   bit1 : 充电完成 (电流降至150mA以下持续3秒)
 *   bit2 : 正在充电
 *   bit3~7: 保留=0
 *
 * alarmOverdischarge 位定义
 *   bit0 : 过放报警 (电压低于保护值确认)
 *   bit1 : 放电完成 (电流降至150mA以下持续3秒)
 *   bit2 : 正在放电
 *   bit3~7: 保留=0
 */

// I2C从机地址 (7位)
#define I2C_SLAVE_ADDR  0x60

// 命令定义
#define CMD_START_DISCHARGE      0x01  // 开始放电 (无数据)
#define CMD_STOP_DISCHARGE       0x02  // 停止放电 (无数据)
#define CMD_START_CHARGE         0x03  // 开始充电 (无数据)
#define CMD_STOP_CHARGE          0x04  // 停止充电 (无数据)
#define CMD_ALARM_OVERCHARGE     0x05  // 读过充报警状态 (读1字节)
#define CMD_ALARM_OVERDISCHARGE  0x06  // 读过放报警状态 (读1字节)
#define CMD_SET_OVERC_PROT_VOLT  0x07  // 设置过充保护电压 (写2字节)
#define CMD_SET_OVERD_PROT_VOLT  0x08  // 设置过放保护电压 (写2字节)
#define CMD_CUR_CHARGE_MAH       0x09  // 读当前充电容量mAh (读2字节)
#define CMD_CUR_DISCHARGE_MAH    0x0A  // 读当前放电容量mAh (读2字节)
#define CMD_CUR_BAT_VOLT         0x0B  // 读当前电池电压mV (读2字节)
#define CMD_READ_OVERC_PROT_VOLT 0x0C  // 读实际过充保护电压 (读2字节)
#define CMD_READ_OVERD_PROT_VOLT 0x0D  // 读实际过放保护电压 (读2字节)
#define CMD_SET_CYCLE_DISCH_NUM  0x0E  // 设置循环放电次数 (写1字节)
#define CMD_CUR_CYCLE_NUM        0x0F  // 读当前循环次数 (读1字节)

// 循环放电结束容量 (每个2字节, 0x10~0x19)
#define CMD_CYCLE_DISCH_END_MAH_1  0x10
#define CMD_CYCLE_DISCH_END_MAH_2  0x11
#define CMD_CYCLE_DISCH_END_MAH_3  0x12
#define CMD_CYCLE_DISCH_END_MAH_4  0x13
#define CMD_CYCLE_DISCH_END_MAH_5  0x14
#define CMD_CYCLE_DISCH_END_MAH_6  0x15
#define CMD_CYCLE_DISCH_END_MAH_7  0x16
#define CMD_CYCLE_DISCH_END_MAH_8  0x17
#define CMD_CYCLE_DISCH_END_MAH_9  0x18
#define CMD_CYCLE_DISCH_END_MAH_10 0x19

// 循环充电结束容量 (每个2字节, 0x1A~0x23)
#define CMD_CYCLE_CHARGE_END_MAH_1  0x1A
#define CMD_CYCLE_CHARGE_END_MAH_2  0x1B
#define CMD_CYCLE_CHARGE_END_MAH_3  0x1C
#define CMD_CYCLE_CHARGE_END_MAH_4  0x1D
#define CMD_CYCLE_CHARGE_END_MAH_5  0x1E
#define CMD_CYCLE_CHARGE_END_MAH_6  0x1F
#define CMD_CYCLE_CHARGE_END_MAH_7  0x20
#define CMD_CYCLE_CHARGE_END_MAH_8  0x21
#define CMD_CYCLE_CHARGE_END_MAH_9  0x22
#define CMD_CYCLE_CHARGE_END_MAH_10 0x23

#define CMD_SET_CHARGE_VOLT     0x24  // 设置充电电压 (写1字节)
#define CMD_SET_CHARGE_CURRENT  0x25  // 设置充电电流 (写1字节)
#define CMD_READ_BAT_VOLT       0x26  // 读电池电压 (读2字节, 同CMD_CUR_BAT_VOLT)
#define CMD_SET_BAT_TYPE        0x27  // 设置电池类型 (写1字节)
#define CMD_CUR_CHARGE_CURRENT    0x28  // 读当前充电电流mA (读2字节)
#define CMD_CUR_DISCHARGE_CURRENT 0x29  // 读当前放电电流mA (读2字节)

// alarmOvercharge 位掩码
#define ALARM_OVERCHARGE_BIT      0x01  // bit0: 过充报警
#define ALARM_CHARGE_COMPLETE_BIT 0x02  // bit1: 充电完成
#define ALARM_CHARGING_BIT        0x04  // bit2: 正在充电
#define ALARM_NO_BATTERY_BIT      0x08  // bit3: 无电池标志

// alarmOverdischarge 位掩码
#define ALARM_OVERDISCHARGE_BIT      0x01  // bit0: 过放报警
#define ALARM_DISCHARGE_COMPLETE_BIT 0x02  // bit1: 放电完成
#define ALARM_DISCHARGING_BIT        0x04  // bit2: 正在放电

// ---------- 外部变量声明 ----------
// 变量存储在xdata区, 在main.c中定义
extern bit i2c_cmd_valid;
extern volatile unsigned char i2c_last_cmd;

extern xdata unsigned int curChargeMAH;            // 当前充电容量 mAh
extern xdata unsigned int curDischargeMAH;         // 当前放电容量 mAh
extern xdata unsigned int curBatVolt;              // 当前电池电压 (mV)
extern xdata unsigned int overchargeProtectVolt;   // 设置的过充保护电压
extern xdata unsigned int overdischargeProtectVolt;// 设置的过放保护电压
extern xdata unsigned int readOverchargeProtectVolt;  // 实际过充保护电压
extern xdata unsigned int readOverdischargeProtectVolt; // 实际过放保护电压
extern xdata unsigned char cycleDischargeNum;        // 循环放电次数
extern xdata unsigned char curCycleNum;              // 当前循环次数
extern xdata unsigned int cycleDischargeEndMAH[10]; // 10次放电结束容量 mAh
extern xdata unsigned int cycleChargeEndMAH[10];   // 10次充电结束容量 mAh
extern xdata unsigned char alarmOvercharge;          // 过充报警位
extern xdata unsigned char alarmOverdischarge;       // 过放报警位
extern xdata unsigned char chargeVoltSet;      // 充电电压设定值
extern xdata unsigned char chargeCurrentSet;   // 充电电流设定值
extern xdata unsigned char batteryType;        // 电池类型
extern xdata unsigned int curChargeCurrentMA;    // 当前充电电流 (mA)
extern xdata unsigned int curDischargeCurrentMA; // 当前放电电流 (mA)

// I2C从机状态变量
extern xdata unsigned char i2c_cmd;                  // 当前收到的命令
extern xdata unsigned char i2c_rx_len;               // 需要接收的数据长度 (0/1/2字节)
extern xdata unsigned char i2c_rx_index;              // 已接收字节数
extern xdata unsigned char i2c_rx_buf[2];             // 接收数据缓冲区
extern xdata unsigned char i2c_tx_len;                // 待发送数据长度
extern xdata unsigned char i2c_tx_index;              // 已发送字节数
extern xdata unsigned char i2c_tx_buf[3];             // 发送缓冲区
extern bit i2c_pending_read;              // 等待读取标志

extern char data_dat;
extern unsigned int adc_bat_code;

// 请求标志
extern bit start_discharge_req;
extern bit pwm_update_req;
extern bit stop_discharge_req;
extern bit start_charge_req;
extern bit stop_charge_req;

// 容量累计变量
extern unsigned long charge_capacity_num;
extern unsigned long discharge_capacity_num;

extern xdata unsigned long charge_capacity_int;   // 充电容量整数部分 (mAh)
extern xdata unsigned long charge_capacity_frac;  // 充电容量小数部分 (mA*ms), < 3600000

extern xdata unsigned long discharge_capacity_int;   // 放电容量整数部分 (mAh)
extern xdata unsigned long discharge_capacity_frac;  // 放电容量小数部分 (mA*ms), < 3600000

#endif
