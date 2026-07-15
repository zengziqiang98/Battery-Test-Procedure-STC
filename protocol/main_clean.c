#include "config.h"
#include "string.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_Timer.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "i2c_cmd.h"
#include "protocol.h"
#include "capacity.h"
#include "STC8H_PWM.h"

#define OVER_VOLT_CONFIRM_MS   50000
#define UNDER_VOLT_CONFIRM_MS  50000
#define CHG_COMPLETE_CONFIRM_MS  30000
#define DISCHG_COMPLETE_CONFIRM_MS 30000

static unsigned long chg_overc_wait_start_tick = 0;
static unsigned char  chg_overc_waiting = 0;
static unsigned long dischg_overd_wait_start_tick = 0;
static unsigned char  dischg_overd_waiting = 0;

unsigned long xdata chg_complete_wait_start_tick = 0;
unsigned char xdata chg_complete_waiting = 0;
unsigned long xdata dischg_complete_wait_start_tick = 0;
unsigned char xdata dischg_complete_waiting = 0;

xdata unsigned int curChargeMAH, curDischargeMAH, curBatVolt;
xdata unsigned int overchargeProtectVolt, overdischargeProtectVolt;
xdata unsigned int readOverchargeProtectVolt, readOverdischargeProtectVolt;
xdata unsigned char cycleDischargeNum, curCycleNum;
xdata unsigned int cycleDischargeEndMAH[10], cycleChargeEndMAH[10];
xdata unsigned char alarmOvercharge, alarmOverdischarge;
xdata unsigned char chargeVoltSet, chargeCurrentSet, batteryType;
xdata unsigned int curChargeCurrentMA, curDischargeCurrentMA;

xdata unsigned char i2c_cmd,i2c_rx_len,i2c_rx_index,i2c_rx_buf[2];
xdata unsigned char i2c_tx_len,i2c_tx_index,i2c_tx_buf[3];
bit i2c_pending_read;
bit start_discharge_req=0,stop_discharge_req=0,start_charge_req=0,stop_charge_req=0;

unsigned long charge_start_tick, discharge_start_tick;
unsigned int CHG_CURRENT_SET_Pwm;
char data_dat=0;

typedef enum { STATE_IDLE,STATE_CHARGING,STATE_DISCHARGING,
               STATE_CHARGE_COMPLETE,STATE_DISCHARGE_COMPLETE } SystemState;
xdata SystemState current_state=STATE_IDLE;
PWMx_Duty PWMA_Duty;

void SetChargeParamsByBatteryType(void){
    switch(batteryType){case 0:CHG_VOLT_SET_1=0;CHG_VOLT_SET_2=0;CHG_CURRENT_SET_Pwm=100;
        PWMA_Duty.PWM1_Duty=24000;overchargeProtectVolt=21500;overdischargeProtectVolt=12000;UpdatePwm(PWMA,&PWMA_Duty);break;
        case 1:CHG_VOLT_SET_1=1;CHG_VOLT_SET_2=1;CHG_CURRENT_SET_Pwm=50;PWMA_Duty.PWM1_Duty=6000;
        overchargeProtectVolt=17500;overdischargeProtectVolt=10000;UpdatePwm(PWMA,&PWMA_Duty);break;
        case 2:CHG_VOLT_SET_1=1;CHG_VOLT_SET_2=1;CHG_CURRENT_SET_Pwm=50;PWMA_Duty.PWM1_Duty=6000;
        overchargeProtectVolt=15000;overdischargeProtectVolt=8500;UpdatePwm(PWMA,&PWMA_Duty);break;
        default:CHG_VOLT_SET_1=0;CHG_VOLT_SET_2=0;CHG_CURRENT_SET=1;break;}}

void PWM_config(void){
    PWMx_InitDefine s;s.PWM_Mode=CCMRn_PWM_MODE1;s.PWM_Duty=1200;s.PWM_EnoSelect=ENO1N;
    PWM_Configuration(PWM1,&s);s.PWM_Period=24000;s.PWM_DeadTime=0;
    s.PWM_MainOutEnable=ENABLE;s.PWM_CEN_Enable=ENABLE;PWM_Configuration(PWMA,&s);
    PWM1_SW(PWM1_SW_P20_P21);NVIC_PWM_Init(PWMA,DISABLE,Priority_0);}

void UART_config(void){
    COMx_InitDefine s;s.UART_Mode=UART_8bit_BRTx;s.UART_BRT_Use=BRT_Timer1;
    s.UART_BaudRate=256000ul;s.UART_RxEnable=ENABLE;UART_Configuration(UART1,&s);
    NVIC_UART1_Init(ENABLE,Priority_1);UART1_SW(UART1_SW_P30_P31);}

void XOSC_Init(void){P_SW2|=0x80;XOSCCR=0xC0;while(!(XOSCCR&0x01));CLKDIV=0x00;CLKSEL=0x01;}

void main(void){
    EAXSFR();XOSC_Init();
    CHARGE_CTRL=0;DISCHARGE_CTRL=0;FAN_CTRL=0;
    charge_capacity_num=0;discharge_capacity_num=0;discharge_capacity_int=0;discharge_capacity_frac=0;
    charge_capacity_int=0;charge_capacity_frac=0;curBatVolt=0;
    overchargeProtectVolt=21000;overdischargeProtectVolt=16000;
    readOverchargeProtectVolt=0;readOverdischargeProtectVolt=0;
    cycleDischargeNum=1;curCycleNum=0;alarmOvercharge=0;alarmOverdischarge=0;
    memset(cycleDischargeEndMAH,0,sizeof(cycleDischargeEndMAH));
    memset(cycleChargeEndMAH,0,sizeof(cycleChargeEndMAH));
    chargeVoltSet=0;chargeCurrentSet=0;batteryType=0;charge_start_tick=0;discharge_start_tick=0;adc_bat_code=0;

    GPIO_config();UART_config();
    {char*m="MBON\r\n";while(*m){SBUF=*m++;while(!TI);TI=0;}}
    EA=1;
    {unsigned char a=0xC0;a|=((unsigned char)ADDR_4<<5);a|=((unsigned char)ADDR_3<<4);a|=((unsigned char)ADDR_2<<3);a|=((unsigned char)ADDR_1<<2);a|=((unsigned char)ADDR_0<<1);Protocol_Init(a>>1);}
    ADC_config();TimeStamp_Init();Capacity_Init();PWM_config();

    while(1){
        Protocol_Poll();Capacity_Update();
        if(stop_discharge_req){stop_discharge_req=0;if(current_state==STATE_DISCHARGING||current_state==STATE_DISCHARGE_COMPLETE){DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_IDLE;}}
        if(stop_charge_req){stop_charge_req=0;if(current_state==STATE_CHARGING||current_state==STATE_CHARGE_COMPLETE){CHARGE_CTRL=0;current_state=STATE_IDLE;}}
        if(start_charge_req){start_charge_req=0;if(current_state==STATE_IDLE){SetChargeParamsByBatteryType();CHARGE_CTRL=1;CHG_CURRENT_SET=1;current_state=STATE_CHARGING;charge_start_tick=TimeStamp_GetTick();}}
        if(start_discharge_req){start_discharge_req=0;if(current_state==STATE_IDLE){FAN_CTRL=1;DISCHARGE_CTRL=1;current_state=STATE_DISCHARGING;discharge_start_tick=TimeStamp_GetTick();}}
        Capacity_Update();
        switch(current_state){
        case STATE_IDLE:CHG_LED=0;DISCHG_LED=0;chg_overc_waiting=0;dischg_overd_waiting=0;curCycleNum=0;break;
        case STATE_CHARGING:CHG_LED=1;DISCHG_LED=0;if((TimeStamp_GetTick()-charge_start_tick)<50000)break;
            if(curBatVolt>=overchargeProtectVolt&&curChargeCurrentMA>300){if(!chg_overc_waiting){chg_overc_waiting=1;chg_overc_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-chg_overc_wait_start_tick)>=OVER_VOLT_CONFIRM_MS){CHARGE_CTRL=0;current_state=STATE_IDLE;chg_overc_waiting=0;}}else chg_overc_waiting=0;
            if(curChargeCurrentMA<=300){if(!chg_complete_waiting){chg_complete_waiting=1;chg_complete_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-chg_complete_wait_start_tick)>=CHG_COMPLETE_CONFIRM_MS){CHARGE_CTRL=0;current_state=STATE_CHARGE_COMPLETE;chg_complete_waiting=0;}}else chg_complete_waiting=0;break;
        case STATE_DISCHARGING:CHG_LED=0;DISCHG_LED=1;if((TimeStamp_GetTick()-discharge_start_tick)<50000)break;
            if(curBatVolt<=overdischargeProtectVolt&&curDischargeCurrentMA>300){if(!dischg_overd_waiting){dischg_overd_waiting=1;dischg_overd_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-dischg_overd_wait_start_tick)>=UNDER_VOLT_CONFIRM_MS){DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_IDLE;dischg_overd_waiting=0;}}else dischg_overd_waiting=0;
            if(curDischargeCurrentMA<=300){if(!dischg_complete_waiting){dischg_complete_waiting=1;dischg_complete_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-dischg_complete_wait_start_tick)>=DISCHG_COMPLETE_CONFIRM_MS){DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_DISCHARGE_COMPLETE;dischg_complete_waiting=0;}}else dischg_complete_waiting=0;break;
        case STATE_CHARGE_COMPLETE:current_state=STATE_IDLE;break;
        case STATE_DISCHARGE_COMPLETE:current_state=STATE_IDLE;break;}
        Protocol_Poll();
    }
}
