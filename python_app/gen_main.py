#!/usr/bin/env python
# Generate main_mcu.c
import os

code = r'''#include "config.h"
#include "string.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_Timer.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "capacity.h"
#include "STC8H_PWM.h"
#include "i2c_cmd.h"
#include <string.h>

#define OVER_VOLT_CONFIRM_MS    50000
#define UNDER_VOLT_CONFIRM_MS   50000
#define CHG_COMPLETE_CONFIRM_MS 30000
#define DISCHG_COMPLETE_CONFIRM_MS 30000

typedef enum { STATE_IDLE, STATE_CHARGING, STATE_DISCHARGING,
               STATE_CHARGE_COMPLETE, STATE_DISCHARGE_COMPLETE } SystemState;
xdata SystemState current_state = STATE_IDLE;

static unsigned long chg_overc_wait_start_tick, dischg_overd_wait_start_tick;
static unsigned char chg_overc_waiting, dischg_overd_waiting;

/* 电池管理全局变量 */
unsigned int CHG_CURRENT_SET_Pwm;
unsigned long charge_start_tick, discharge_start_tick;
xdata unsigned char chg_complete_waiting, dischg_complete_waiting;
xdata unsigned long chg_complete_wait_start_tick, dischg_complete_wait_start_tick;

/* UART2 ring buffer */
#define RB2_SIZE 256
static unsigned char rb2[RB2_SIZE];
static volatile unsigned char rb2_wr, rb2_rd, rb2_cnt;

void SetChargeParamsByBatteryType(void)
{
    switch (batteryType) {
        case 0: CHG_VOLT_SET_1=0; CHG_VOLT_SET_2=0; CHG_CURRENT_SET_Pwm=100;
                overchargeProtectVolt=21500; overdischargeProtectVolt=12000; break;
        case 1: CHG_VOLT_SET_1=1; CHG_VOLT_SET_2=1; CHG_CURRENT_SET_Pwm=50;
                overchargeProtectVolt=17500; overdischargeProtectVolt=10000; break;
        case 2: CHG_VOLT_SET_1=1; CHG_VOLT_SET_2=1; CHG_CURRENT_SET_Pwm=50;
                overchargeProtectVolt=15000; overdischargeProtectVolt=8500; break;
    }
}

void UART2_Init(void)
{
    COMx_InitDefine s;
    s.UART_Mode = UART_8bit_BRTx;
    s.UART_BRT_Use = BRT_Timer2;
    s.UART_BaudRate = 115200ul;
    s.UART_RxEnable = ENABLE;
    UART_Configuration(UART2, &s);
    NVIC_UART2_Init(ENABLE, Priority_1);
    UART2_SW(UART2_SW_P10_P11);
    rb2_wr = 0; rb2_rd = 0; rb2_cnt = 0;
}

void XOSC_Init(void)
{
    P_SW2 |= 0x80; XOSCCR = 0xC0;
    while (!(XOSCCR & 0x01));
    CLKDIV = 0x00; CLKSEL = 0x01;
}

void PWM_config(void)
{
    PWMx_InitDefine s;
    s.PWM_Mode = CCMRn_PWM_MODE1; s.PWM_Duty = 1200;
    s.PWM_EnoSelect = ENO1N;
    PWM_Configuration(PWM1, &s);
    s.PWM_Period = 24000; s.PWM_DeadTime = 0;
    s.PWM_MainOutEnable = ENABLE; s.PWM_CEN_Enable = ENABLE;
    PWM_Configuration(PWMA, &s);
    PWM1_SW(PWM1_SW_P20_P21);
    NVIC_PWM_Init(PWMA, DISABLE, Priority_0);
}

void UART2_ISR(void) interrupt 8
{
    if (S2CON & 0x01) {
        S2CON &= ~0x01;
        rb2[rb2_wr++] = S2BUF;
        if (rb2_wr >= RB2_SIZE) rb2_wr = 0;
        if (rb2_cnt < RB2_SIZE) rb2_cnt++;
        else { rb2_rd = rb2_wr; rb2_cnt = RB2_SIZE; }
    }
    if (S2CON & 0x02) {
        S2CON &= ~0x02;
    }
}

void SendRegs(void)
{
    unsigned char d[23], chk, i;
    unsigned short val;
    for (i = 0; i < 11; i++) {
        switch (i) {
            case 0:  val = alarmOvercharge; break;
            case 1:  val = alarmOverdischarge; break;
            case 2:  val = curChargeMAH; break;
            case 3:  val = curDischargeMAH; break;
            case 4:  val = curBatVolt; break;
            case 5:  val = curChargeCurrentMA; break;
            case 6:  val = curDischargeCurrentMA; break;
            case 7:  val = overchargeProtectVolt; break;
            case 8:  val = overdischargeProtectVolt; break;
            case 9:  val = curCycleNum; break;
            case 10: val = 0x0101; break;
            default: val = 0; break;
        }
        d[i*2]   = (unsigned char)(val >> 8);
        d[i*2+1] = (unsigned char)(val & 0xFF);
    }
    chk = 0;
    for (i = 0; i < 22; i++) chk ^= d[i];
    d[22] = chk;
    for (i = 0; i < 23; i++) {
        S2BUF = d[i];
        while (!(S2CON & 0x02));
        S2CON &= ~0x02;
    }
}

void HandleCmd(unsigned char *frm, unsigned char len)
{
    unsigned short addr, val;
    if (len < 7) return;
    addr = ((unsigned short)frm[3] << 8) | frm[4];
    val  = ((unsigned short)frm[5] << 8) | frm[6];
    switch (addr) {
        case 0x0000:
            switch (val) {
                case 0x0001: start_discharge_req = 1; break;
                case 0x0002: stop_discharge_req  = 1; break;
                case 0x0003: start_charge_req    = 1; break;
                case 0x0004: stop_charge_req     = 1; break;
                case 0x0005: stop_charge_req = 1; stop_discharge_req = 1; break;
            } break;
        case 0x0010: overchargeProtectVolt = val; break;
        case 0x0011: overdischargeProtectVolt = val; break;
        case 0x0012: cycleDischargeNum = (unsigned char)val; break;
        case 0x0013:
            chargeVoltSet = (unsigned char)val;
            CHG_VOLT_SET_1 = (chargeVoltSet & 1) ? 1 : 0;
            CHG_VOLT_SET_2 = (chargeVoltSet & 2) ? 1 : 0; break;
        case 0x0014: chargeCurrentSet = (unsigned char)val; CHG_CURRENT_SET = 1; break;
        case 0x0015: batteryType = (unsigned char)val; SetChargeParamsByBatteryType(); break;
    }
}

void UART1_Init(void)
{
    COMx_InitDefine s;
    s.UART_Mode = UART_8bit_BRTx;
    s.UART_BRT_Use = BRT_Timer1;
    s.UART_BaudRate = 115200ul;
    s.UART_RxEnable = ENABLE;
    UART_Configuration(UART1, &s);
    NVIC_UART1_Init(DISABLE, Priority_0);
    UART1_SW(UART1_SW_P30_P31);
}

void main(void)
{
    EAXSFR(); XOSC_Init();
    CHARGE_CTRL = 0; DISCHARGE_CTRL = 0; FAN_CTRL = 0;

    charge_capacity_num = 0; discharge_capacity_num = 0;
    discharge_capacity_int = 0; discharge_capacity_frac = 0;
    charge_capacity_int = 0; charge_capacity_frac = 0;
    curBatVolt = 0;
    overchargeProtectVolt = 21000; overdischargeProtectVolt = 16000;
    readOverchargeProtectVolt = 0; readOverdischargeProtectVolt = 0;
    cycleDischargeNum = 1; curCycleNum = 0;
    alarmOvercharge = 0; alarmOverdischarge = 0;
    memset(cycleDischargeEndMAH, 0, 20);
    memset(cycleChargeEndMAH, 0, 20);
    chargeVoltSet = 0; chargeCurrentSet = 0; batteryType = 0;
    charge_start_tick = 0; discharge_start_tick = 0;
    chg_overc_waiting = 0; dischg_overd_waiting = 0;
    chg_complete_waiting = 0; dischg_complete_waiting = 0;

    GPIO_config(); UART1_Init();
    ES = 0;  /* 关串口中断, MBON用轮询 */
    { char *m = "MBON\r\n"; while (*m) { SBUF = *m++; while (!TI); TI = 0; } }
    EA = 1;
    /* ADC_config(); TimeStamp_Init(); Capacity_Init(); PWM_config(); */

    while (1)
    {
        /* 极简测试: 只验证活着 */
        {
            static unsigned long last = 0;
            if (TimeStamp_GetTick() - last > 5000) {
                char *m = "OK\r\n"; while (*m) { SBUF = *m++; while (!TI); TI = 0; }
                last = TimeStamp_GetTick();
            }
        }
#if 0
        if (rb2_cnt > 0) {
            unsigned char frm[64], i, start, flen, b0, len;
            start = (rb2_wr >= rb2_cnt) ? (rb2_wr - rb2_cnt) : (RB2_SIZE + rb2_wr - rb2_cnt);
            b0 = rb2[(start) % RB2_SIZE];
            if (b0 == 0x01 && rb2_cnt >= 1) {
                rb2_cnt = 0;
                SendRegs();
            } else if (rb2_cnt >= 4) {
                len = rb2[(start + 2) % RB2_SIZE];
                flen = 4 + len;
                if (flen > 64) flen = 64;
                if (rb2_cnt >= flen) {
                    for (i = 0; i < flen; i++) frm[i] = rb2[(start + i) % RB2_SIZE];
                    rb2_cnt = 0;
                    HandleCmd(frm, flen);
                }
            }
        }

        { static unsigned char d = 0; if (++d >= 10) { d = 0; Capacity_Update(); } }

        if (stop_discharge_req) {
            stop_discharge_req = 0;
            if (current_state == STATE_DISCHARGING || current_state == STATE_DISCHARGE_COMPLETE)
                { DISCHARGE_CTRL = 0; FAN_CTRL = 0; current_state = STATE_IDLE; }
        }
        if (stop_charge_req) {
            stop_charge_req = 0;
            if (current_state == STATE_CHARGING || current_state == STATE_CHARGE_COMPLETE)
                { CHARGE_CTRL = 0; current_state = STATE_IDLE; }
        }
        if (start_charge_req) {
            start_charge_req = 0;
            if (current_state == STATE_IDLE)
                { SetChargeParamsByBatteryType(); CHARGE_CTRL = 1; CHG_CURRENT_SET = 1;
                  current_state = STATE_CHARGING; charge_start_tick = TimeStamp_GetTick(); }
        }
        if (start_discharge_req) {
            start_discharge_req = 0;
            if (current_state == STATE_IDLE)
                { FAN_CTRL = 1; DISCHARGE_CTRL = 1;
                  current_state = STATE_DISCHARGING; discharge_start_tick = TimeStamp_GetTick(); }
        }

        switch (current_state) {
        case STATE_IDLE:
            CHG_LED = 0; DISCHG_LED = 0;
            break;
        case STATE_CHARGING:
            CHG_LED = 1; DISCHG_LED = 0;
            if ((TimeStamp_GetTick() - charge_start_tick) < 50000) break;
            if (curBatVolt >= overchargeProtectVolt && curChargeCurrentMA > 300) {
                if (!chg_overc_waiting) { chg_overc_waiting = 1; chg_overc_wait_start_tick = TimeStamp_GetTick(); }
                else if ((TimeStamp_GetTick() - chg_overc_wait_start_tick) >= OVER_VOLT_CONFIRM_MS)
                    { CHARGE_CTRL = 0; current_state = STATE_IDLE; chg_overc_waiting = 0; }
            } else chg_overc_waiting = 0;
            if (curChargeCurrentMA <= 300) {
                if (!chg_complete_waiting) { chg_complete_waiting = 1; chg_complete_wait_start_tick = TimeStamp_GetTick(); }
                else if ((TimeStamp_GetTick() - chg_complete_wait_start_tick) >= CHG_COMPLETE_CONFIRM_MS)
                    { CHARGE_CTRL = 0; current_state = STATE_CHARGE_COMPLETE; chg_complete_waiting = 0; }
            } else chg_complete_waiting = 0;
            break;
        case STATE_DISCHARGING:
            CHG_LED = 0; DISCHG_LED = 1;
            if ((TimeStamp_GetTick() - discharge_start_tick) < 50000) break;
            if (curBatVolt <= overdischargeProtectVolt && curDischargeCurrentMA > 300) {
                if (!dischg_overd_waiting) { dischg_overd_waiting = 1; dischg_overd_wait_start_tick = TimeStamp_GetTick(); }
                else if ((TimeStamp_GetTick() - dischg_overd_wait_start_tick) >= UNDER_VOLT_CONFIRM_MS)
                    { DISCHARGE_CTRL = 0; FAN_CTRL = 0; current_state = STATE_IDLE; dischg_overd_waiting = 0; }
            } else dischg_overd_waiting = 0;
            if (curDischargeCurrentMA <= 300) {
                if (!dischg_complete_waiting) { dischg_complete_waiting = 1; dischg_complete_wait_start_tick = TimeStamp_GetTick(); }
                else if ((TimeStamp_GetTick() - dischg_complete_wait_start_tick) >= DISCHG_COMPLETE_CONFIRM_MS)
                    { DISCHARGE_CTRL = 0; FAN_CTRL = 0; current_state = STATE_DISCHARGE_COMPLETE; dischg_complete_waiting = 0; }
            } else dischg_complete_waiting = 0;
            break;
        case STATE_CHARGE_COMPLETE:  current_state = STATE_IDLE; break;
        case STATE_DISCHARGE_COMPLETE: current_state = STATE_IDLE; break;
        }
#endif
    }
}
'''

dst = r'D:\电池测试治具2026-4-21\电池测试治具2025-11-14\main_mcu.c'
with open(dst, 'wb') as f:
    f.write(code.encode('ascii', errors='replace'))
print('main_mcu.c generated')
