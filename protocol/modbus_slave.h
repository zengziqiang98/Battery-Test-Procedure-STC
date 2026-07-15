/**
 * @file    modbus_slave.h
 * @brief   Modbus RTU 从站协议栈 (硬件定时器 t3.5 检测版)
 */

#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "config.h"

/* ========== Modbus 寄存器地址 ========== */
#define MB_REG_CMD                  0x0000
#define MB_REG_STATUS_CHARGE        0x0001
#define MB_REG_STATUS_DISCHARGE     0x0002
#define MB_REG_CHARGE_MAH           0x0003
#define MB_REG_DISCHARGE_MAH        0x0004
#define MB_REG_BAT_VOLT             0x0005
#define MB_REG_CHARGE_CURRENT       0x0006
#define MB_REG_DISCHARGE_CURRENT    0x0007
#define MB_REG_OVERC_PROT           0x0008
#define MB_REG_OVERD_PROT           0x0009
#define MB_REG_CYCLE_NUM            0x000A
#define MB_REG_DEVICE_ID            0x000B
#define MB_REG_SET_OVERC_PROT       0x0010
#define MB_REG_SET_OVERD_PROT       0x0011
#define MB_REG_SET_CYCLE_NUM        0x0012
#define MB_REG_SET_CHG_VOLT         0x0013
#define MB_REG_SET_CHG_CURRENT      0x0014
#define MB_REG_SET_BAT_TYPE         0x0015
#define MB_REG_CYCLE_DISCH_BASE     0x0020
#define MB_REG_CYCLE_CHG_BASE       0x0030

/* ========== 命令码 ========== */
#define MB_CMD_START_DISCHARGE      0x0001
#define MB_CMD_STOP_DISCHARGE       0x0002
#define MB_CMD_START_CHARGE         0x0003
#define MB_CMD_STOP_CHARGE          0x0004
#define MB_CMD_EMERGENCY_STOP       0x0005
#define MB_CMD_BUZZER_ON            0x0006
#define MB_CMD_BUZZER_OFF           0x0007

/* ========== 设备ID ========== */
#define MB_DEVICE_ID                0x0101

/* ========== API ========== */
void Modbus_Init(unsigned char addr);
void Modbus_Poll(void);
void Modbus_RxByte(unsigned char ch);
void Modbus_TimerT35_ISR(void);    /* 在定时器中断中调用 */
unsigned char Modbus_GetAddr(void);

#endif
