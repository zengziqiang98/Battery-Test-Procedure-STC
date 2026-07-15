/**
 * comm_mcu.c - 通讯MCU: RS-485(PC) ↔ UART2(主MCU), 带数据缓存
 *
 * UART1(P3.0/3.1): PC/RS-485  RI轮询 TX: SBUF
 * UART2(P1.0/1.1): 主MCU       S2RI轮询 TX: S2BUF
 *
 * 工作方式:
 *   后台每50ms向主MCU拉一次数据(22B寄存器), 存本地缓存
 *   PC读请求 → 直接从缓存回复 (微秒级)
 *   PC写/命令 → 转发给主MCU
 */

#include "config.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "STC8G_H_Timer.h"
#include "i2c_cmd.h"
#include <string.h>

/* 哑变量 */
xdata unsigned int curChargeMAH,curDischargeMAH,curBatVolt;
xdata unsigned int overchargeProtectVolt,overdischargeProtectVolt;
xdata unsigned int readOverchargeProtectVolt,readOverdischargeProtectVolt;
xdata unsigned int curChargeCurrentMA,curDischargeCurrentMA;
xdata unsigned int cycleDischargeEndMAH[10],cycleChargeEndMAH[10];
xdata unsigned char cycleDischargeNum,curCycleNum;
xdata unsigned char alarmOvercharge,alarmOverdischarge;
xdata unsigned char chargeVoltSet,chargeCurrentSet,batteryType;
xdata unsigned char chg_complete_waiting,dischg_complete_waiting;
xdata unsigned long chg_complete_wait_start_tick,dischg_complete_wait_start_tick;
bit start_discharge_req,stop_discharge_req,start_charge_req,stop_charge_req;
xdata unsigned char i2c_cmd,i2c_rx_len,i2c_rx_index,i2c_rx_buf[2];
xdata unsigned char i2c_tx_len,i2c_tx_index,i2c_tx_buf[3];
bit i2c_pending_read,i2c_cmd_valid,pwm_update_req;
volatile unsigned char i2c_last_cmd;
unsigned long charge_start_tick,discharge_start_tick;
unsigned int CHG_CURRENT_SET_Pwm;char data_dat;
unsigned long charge_capacity_num,discharge_capacity_num;
unsigned long discharge_capacity_int,discharge_capacity_frac;
unsigned long charge_capacity_int,charge_capacity_frac;
unsigned int adc_bat_code;
void SetChargeParamsByBatteryType(void){}
void PWM_config(void){}

/* 本地缓存: 22字节寄存器数据 */
static unsigned char cache[22];
static unsigned char my_addr;

static unsigned char xor_chk(unsigned char *d,unsigned char n){
    unsigned char c=0,i;for(i=0;i<n;i++)c^=d[i];return c;
}

void UART1_Init(void){
    COMx_InitDefine s;
    s.UART_Mode=UART_8bit_BRTx;s.UART_BRT_Use=BRT_Timer1;
    s.UART_BaudRate=256000ul;s.UART_RxEnable=ENABLE;
    UART_Configuration(UART1,&s);
    NVIC_UART1_Init(DISABLE,Priority_0);
    UART1_SW(UART1_SW_P30_P31);
}

void UART2_Init(void){
    COMx_InitDefine s;
    s.UART_Mode=UART_8bit_BRTx;s.UART_BRT_Use=BRT_Timer2;
    s.UART_BaudRate=256000ul;s.UART_RxEnable=ENABLE;
    UART_Configuration(UART2,&s);
    NVIC_UART2_Init(DISABLE,Priority_0);
    UART2_SW(UART2_SW_P10_P11);
}

void XOSC_Init(void){P_SW2|=0x80;XOSCCR=0xC0;while(!(XOSCCR&0x01));CLKDIV=0x00;CLKSEL=0x01;}

void GetHWAddr(unsigned char *a){
    unsigned char hw=0;
    hw|=(ADDR_0)?1:0;hw|=(ADDR_1)?2:0;hw|=(ADDR_2)?4:0;
    hw|=(ADDR_3)?8:0;hw|=(ADDR_4)?16:0;*a=hw+1;
}

/* 从主MCU拉取数据并更新缓存 */
void FetchFromMain(void){
    unsigned char i,chk;
    S2BUF=0x01;while(!(S2CON&0x02));S2CON&=~0x02;  /* 发送查询命令0x01 */
    /* 等23字节回复: 22数据+1校验 */
    {
        unsigned long t0=TimeStamp_GetTick();
        unsigned char pos=0,data[23];
        while(pos<23&&(TimeStamp_GetTick()-t0)<50){
            if(S2CON&0x01){S2CON&=~0x01;data[pos++]=S2BUF;}
        }
        if(pos>=23&&xor_chk(data,22)==data[22])
            for(i=0;i<22;i++)cache[i]=data[i];  /* 更新缓存 */
    }
}

/* 从缓存构建回复并通过UART1发送 */
void ReplyFromCache(unsigned char addr,unsigned char cmd){
    unsigned char buf[32],chk,i;
    buf[0]=addr;buf[1]=cmd|0x80;buf[2]=0;buf[3]=22;
    for(i=0;i<22;i++)buf[4+i]=cache[i];
    chk=xor_chk(buf,26);buf[26]=chk;
    for(i=0;i<27;i++){SBUF=buf[i];while(!TI);TI=0;}
}

/* 转发帧到主MCU (通过UART2) */
void ForwardToMain(unsigned char *frm,unsigned char len){
    unsigned char i;
    for(i=0;i<len;i++){S2BUF=frm[i];while(!(S2CON&0x02));S2CON&=~0x02;}
}

/* 发ACK回PC */
void SendAck(unsigned char addr,unsigned char cmd){
    unsigned char ack[5],chk;
    ack[0]=addr;ack[1]=cmd|0x80;ack[2]=0;ack[3]=0;
    chk=xor_chk(ack,4);ack[4]=chk;
    for(unsigned char i=0;i<5;i++){SBUF=ack[i];while(!TI);TI=0;}
}

void main(void){
    unsigned char i,rx_buf[64],rx_len=0,ch,cmd,len;
    unsigned long last_fetch=0,now;

    EAXSFR();XOSC_Init();
    CHARGE_CTRL=0;DISCHARGE_CTRL=0;FAN_CTRL=0;
    GPIO_config();UART1_Init();UART2_Init();
    GetHWAddr(&my_addr);
    {char *m="MBON\r\n";while(*m){SBUF=*m++;while(!TI);TI=0;}}
    EA=1;
    TimeStamp_Init();

    while(1){
        /* 每50ms从主MCU拉取数据 */
        now=TimeStamp_GetTick();
        if((now-last_fetch)>=500){FetchFromMain();last_fetch=now;}

        /* UART1(PC侧)收帧: RI轮询 */
        while(RI){ch=SBUF;RI=0;if(rx_len<60)rx_buf[rx_len++]=ch;else rx_len=0;}
        if(rx_len<4)continue;
        len=rx_buf[2];if(rx_len<4+len)continue;

        /* 检查地址 */
        if(rx_buf[0]!=my_addr&&rx_buf[0]!=0){rx_len=0;continue;}

        /* 帧收齐, 处理 */
        i=rx_len;rx_len=0;
        /* XOR校验 */
        if(xor_chk(rx_buf,i-1)!=rx_buf[i-1])continue;

        cmd=rx_buf[1];len=rx_buf[2];

        if(rx_buf[0]==0){  /* 广播: 转发 */
            ForwardToMain(rx_buf,i);continue;
        }

        switch(cmd){
        case 0x03:  /* 读请求 → 缓存直接回复 */
            ReplyFromCache(my_addr,cmd);break;
        case 0x06:  /* 写/命令 → 转发+ACK */
            ForwardToMain(rx_buf,i);
            SendAck(my_addr,cmd);break;
        default: break;
        }
    }
}
