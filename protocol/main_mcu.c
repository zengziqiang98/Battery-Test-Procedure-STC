/**
 * main_mcu.c - 主MCU: 电池管理 + UART2响应通讯MCU
 *
 * UART2(P1.0/1.1): 与通讯MCU通信
 *   收到 0x01 → 回复 22字节寄存器数据 + XOR
 *   收到帧 → 执行写/控制命令
 */

#include "config.h"
#include "string.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_Timer.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "capacity.h"
#include "STC8H_PWM.h"
#include "i2c_cmd.h"

#define OVER_VOLT_CONFIRM_MS 50000
#define UNDER_VOLT_CONFIRM_MS 50000
#define CHG_COMPLETE_CONFIRM_MS 30000
#define DISCHG_COMPLETE_CONFIRM_MS 30000

typedef enum{STATE_IDLE,STATE_CHARGING,STATE_DISCHARGING,
    STATE_CHARGE_COMPLETE,STATE_DISCHARGE_COMPLETE}SystemState;
xdata SystemState current_state=STATE_IDLE;
static unsigned long chg_overc_wait_start_tick,dischg_overd_wait_start_tick;
static unsigned char chg_overc_waiting,dischg_overd_waiting;
unsigned int CHG_CURRENT_SET_Pwm;
unsigned long charge_start_tick,discharge_start_tick;
xdata unsigned char chg_complete_waiting,dischg_complete_waiting;
xdata unsigned long chg_complete_wait_start_tick,dischg_complete_wait_start_tick;

void SetChargeParamsByBatteryType(void){
    switch(batteryType){case 0:CHG_VOLT_SET_1=0;CHG_VOLT_SET_2=0;CHG_CURRENT_SET_Pwm=100;
        overchargeProtectVolt=21500;overdischargeProtectVolt=12000;break;
        case 1:CHG_VOLT_SET_1=1;CHG_VOLT_SET_2=1;CHG_CURRENT_SET_Pwm=50;
        overchargeProtectVolt=17500;overdischargeProtectVolt=10000;break;
        case 2:CHG_VOLT_SET_1=1;CHG_VOLT_SET_2=1;CHG_CURRENT_SET_Pwm=50;
        overchargeProtectVolt=15000;overdischargeProtectVolt=8500;break;}}

void UART2_Init(void){
    COMx_InitDefine s;
    s.UART_Mode=UART_8bit_BRTx;s.UART_BRT_Use=BRT_Timer2;
    s.UART_BaudRate=256000ul;s.UART_RxEnable=ENABLE;
    UART_Configuration(UART2,&s);
    NVIC_UART2_Init(DISABLE,Priority_0);
    UART2_SW(UART2_SW_P10_P11);
}

void XOSC_Init(void){P_SW2|=0x80;XOSCCR=0xC0;while(!(XOSCCR&0x01));CLKDIV=0x00;CLKSEL=0x01;}

void PWM_config(void){
    PWMx_InitDefine s;
    s.PWM_Mode=CCMRn_PWM_MODE1;s.PWM_Duty=1200;s.PWM_EnoSelect=ENO1N;
    PWM_Configuration(PWM1,&s);
    s.PWM_Period=24000;s.PWM_DeadTime=0;s.PWM_MainOutEnable=ENABLE;s.PWM_CEN_Enable=ENABLE;
    PWM_Configuration(PWMA,&s);
    PWM1_SW(PWM1_SW_P20_P21);
    NVIC_PWM_Init(PWMA,DISABLE,Priority_0);
}

/* 回复通讯MCU: 22字节寄存器 + XOR */
void SendRegs(void){
    unsigned char d[23],chk,i;unsigned short v;
    for(i=0;i<11;i++){switch(i){case 0:v=alarmOvercharge;break;case 1:v=alarmOverdischarge;break;
        case 2:v=curChargeMAH;break;case 3:v=curDischargeMAH;break;case 4:v=curBatVolt;break;
        case 5:v=curChargeCurrentMA;break;case 6:v=curDischargeCurrentMA;break;
        case 7:v=overchargeProtectVolt;break;case 8:v=overdischargeProtectVolt;break;
        case 9:v=curCycleNum;break;case 10:v=0x0101;break;}
        d[i*2]=(unsigned char)(v>>8);d[i*2+1]=(unsigned char)(v&0xFF);}
    chk=0;for(i=0;i<22;i++)chk^=d[i];d[22]=chk;
    for(i=0;i<23;i++){S2BUF=d[i];while(!(S2CON&0x02));S2CON&=~0x02;}
}

static unsigned char xor_chk(unsigned char *d,unsigned char n){
    unsigned char c=0,i;for(i=0;i<n;i++)c^=d[i];return c;}

/* 处理命令帧: [ADDR][CMD][LEN][DATA...][CHK] */
void HandleCmd(unsigned char *frm){
    unsigned short addr,val;
    addr=((unsigned short)frm[3]<<8)|frm[4];
    val=((unsigned short)frm[5]<<8)|frm[6];
    switch(addr){case 0x0000:switch(val){case 0x0001:start_discharge_req=1;break;
        case 0x0002:stop_discharge_req=1;break;case 0x0003:start_charge_req=1;break;
        case 0x0004:stop_charge_req=1;break;case 0x0005:stop_charge_req=1;stop_discharge_req=1;break;}break;
        case 0x0010:overchargeProtectVolt=val;break;case 0x0011:overdischargeProtectVolt=val;break;
        case 0x0012:cycleDischargeNum=(unsigned char)val;break;
        case 0x0013:chargeVoltSet=(unsigned char)val;
            CHG_VOLT_SET_1=(chargeVoltSet&1)?1:0;CHG_VOLT_SET_2=(chargeVoltSet&2)?1:0;break;
        case 0x0014:chargeCurrentSet=(unsigned char)val;CHG_CURRENT_SET=1;break;
        case 0x0015:batteryType=(unsigned char)val;SetChargeParamsByBatteryType();break;}
}

void main(void){
    unsigned char u2_buf[64],u2_len=0,ch,flen,i;
    EAXSFR();XOSC_Init();
    CHARGE_CTRL=0;DISCHARGE_CTRL=0;FAN_CTRL=0;
    charge_capacity_num=0;discharge_capacity_num=0;
    discharge_capacity_int=0;discharge_capacity_frac=0;
    charge_capacity_int=0;charge_capacity_frac=0;curBatVolt=0;
    overchargeProtectVolt=21000;overdischargeProtectVolt=16000;
    readOverchargeProtectVolt=0;readOverdischargeProtectVolt=0;
    cycleDischargeNum=1;curCycleNum=0;alarmOvercharge=0;alarmOverdischarge=0;
    memset(cycleDischargeEndMAH,0,20);memset(cycleChargeEndMAH,0,20);
    chargeVoltSet=0;chargeCurrentSet=0;batteryType=0;
    charge_start_tick=0;discharge_start_tick=0;
    chg_overc_waiting=0;dischg_overd_waiting=0;
    chg_complete_waiting=0;dischg_complete_waiting=0;
    GPIO_config();UART2_Init();
    {char *m="MBON\r\n";while(*m){SBUF=*m++;while(!TI);TI=0;}}
    EA=1;
    ADC_config();TimeStamp_Init();Capacity_Init();PWM_config();

    while(1){
        /* UART2收数据: S2RI轮询 */
        while(S2CON&0x01){ch=S2BUF;S2CON&=~0x01;
            if(u2_len<60)u2_buf[u2_len++]=ch;else u2_len=0;}

        if(u2_len>0){
            if(u2_buf[0]==0x01&&u2_len>=1){  /* 数据拉取命令 */
                u2_len=0;SendRegs();
            }else if(u2_len>=4){  /* 控制命令帧 */
                flen=4+u2_buf[2];if(flen>64)flen=64;
                if(u2_len>=flen&&xor_chk(u2_buf,flen-1)==u2_buf[flen-1]){
                    HandleCmd(u2_buf);u2_len=0;}
            }
        }

        /* 电池管理 */
        {static unsigned char d=0;if(++d>=10){d=0;Capacity_Update();}}
        if(stop_discharge_req){stop_discharge_req=0;
            if(current_state==STATE_DISCHARGING||current_state==STATE_DISCHARGE_COMPLETE)
            {DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_IDLE;}}
        if(stop_charge_req){stop_charge_req=0;
            if(current_state==STATE_CHARGING||current_state==STATE_CHARGE_COMPLETE)
            {CHARGE_CTRL=0;current_state=STATE_IDLE;}}
        if(start_charge_req){start_charge_req=0;
            if(current_state==STATE_IDLE){SetChargeParamsByBatteryType();
                CHARGE_CTRL=1;CHG_CURRENT_SET=1;current_state=STATE_CHARGING;
                charge_start_tick=TimeStamp_GetTick();}}
        if(start_discharge_req){start_discharge_req=0;
            if(current_state==STATE_IDLE){FAN_CTRL=1;DISCHARGE_CTRL=1;
                current_state=STATE_DISCHARGING;discharge_start_tick=TimeStamp_GetTick();}}

        switch(current_state){
        case STATE_IDLE:CHG_LED=0;DISCHG_LED=0;break;
        case STATE_CHARGING:CHG_LED=1;DISCHG_LED=0;
            if((TimeStamp_GetTick()-charge_start_tick)<50000)break;
            if(curBatVolt>=overchargeProtectVolt&&curChargeCurrentMA>300)
            {if(!chg_overc_waiting){chg_overc_waiting=1;chg_overc_wait_start_tick=TimeStamp_GetTick();}
            else if((TimeStamp_GetTick()-chg_overc_wait_start_tick)>=OVER_VOLT_CONFIRM_MS)
            {CHARGE_CTRL=0;current_state=STATE_IDLE;chg_overc_waiting=0;}}else chg_overc_waiting=0;
            if(curChargeCurrentMA<=300){if(!chg_complete_waiting){chg_complete_waiting=1;
            chg_complete_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-chg_complete_wait_start_tick)>=CHG_COMPLETE_CONFIRM_MS)
            {CHARGE_CTRL=0;current_state=STATE_CHARGE_COMPLETE;chg_complete_waiting=0;}}else chg_complete_waiting=0;break;
        case STATE_DISCHARGING:CHG_LED=0;DISCHG_LED=1;
            if((TimeStamp_GetTick()-discharge_start_tick)<50000)break;
            if(curBatVolt<=overdischargeProtectVolt&&curDischargeCurrentMA>300)
            {if(!dischg_overd_waiting){dischg_overd_waiting=1;dischg_overd_wait_start_tick=TimeStamp_GetTick();}
            else if((TimeStamp_GetTick()-dischg_overd_wait_start_tick)>=UNDER_VOLT_CONFIRM_MS)
            {DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_IDLE;dischg_overd_waiting=0;}}else dischg_overd_waiting=0;
            if(curDischargeCurrentMA<=300){if(!dischg_complete_waiting){dischg_complete_waiting=1;
            dischg_complete_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-dischg_complete_wait_start_tick)>=DISCHG_COMPLETE_CONFIRM_MS)
            {DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_DISCHARGE_COMPLETE;dischg_complete_waiting=0;}}else dischg_complete_waiting=0;break;
        case STATE_CHARGE_COMPLETE:current_state=STATE_IDLE;break;
        case STATE_DISCHARGE_COMPLETE:current_state=STATE_IDLE;break;
        }
    }
}
