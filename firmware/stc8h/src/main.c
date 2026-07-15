/**
 * modbus_main.c - Modbus RTU Slave 基于 STC 官方例程模板
 *
 * 依赖: STC8G_H_UART_Isr.c (中断收发)
 *       capacity.c (ADC + TimeStamp)
 *       ADC.c (整数版 ADC)
 */

#include "config.h"
#include "string.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_I2C.h"
#include "STC8G_H_Delay.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "STC8G_H_Timer.h"
#include "capacity.h"
#include "STC8H_PWM.h"
#include "i2c_cmd.h"

#define EXECUTOR_DEBUG_UART 0

void SetChargeParamsByBatteryType(void);

xdata unsigned int curChargeMAH;
xdata unsigned int curDischargeMAH;
xdata unsigned int curBatVolt;
xdata unsigned int overchargeProtectVolt;
xdata unsigned int overdischargeProtectVolt;
xdata unsigned int readOverchargeProtectVolt;
xdata unsigned int readOverdischargeProtectVolt;
xdata unsigned char cycleDischargeNum;
xdata unsigned char curCycleNum;
xdata unsigned int cycleDischargeEndMAH[10];
xdata unsigned int cycleChargeEndMAH[10];
xdata unsigned char alarmOvercharge;
xdata unsigned char alarmOverdischarge;
xdata unsigned char chargeVoltSet;
xdata unsigned char chargeCurrentSet;
xdata unsigned char batteryType;
xdata unsigned int curChargeCurrentMA;
xdata unsigned int curDischargeCurrentMA;

xdata unsigned char i2c_cmd;
xdata unsigned char i2c_rx_len;
xdata unsigned char i2c_rx_index;
xdata unsigned char i2c_rx_buf[2];
xdata unsigned char i2c_tx_len;
xdata unsigned char i2c_tx_index;
xdata unsigned char i2c_tx_buf[3];
bit i2c_pending_read;

bit start_discharge_req = 0;
bit stop_discharge_req = 0;
bit start_charge_req = 0;
bit stop_charge_req = 0;
char data_dat = 0;

/* ===== CRC16 (Modbus 标准, 多项式 0xA001) ===== */
static const unsigned char code crcHi[] = {
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40
};
static const unsigned char code crcLo[] = {
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
    0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,0x08,0xC8,
    0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,
    0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,
    0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,0xFB,0x39,0xF9,0xF8,0x38,
    0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,
    0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,
    0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,
    0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,
    0xB4,0x74,0x75,0xB5,0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,
    0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
    0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,
    0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
};
static unsigned short crc16(u8 *buf, u8 n)
{
    u8 hi=0xFF,lo=0xFF,i,idx;
    for(i=0;i<n;i++){idx=lo^buf[i];lo=hi^crcHi[idx];hi=crcLo[idx];}
    return ((unsigned short)hi<<8)|lo;
}

static u8 module_addr=1;

static u8 get_hw_addr(void)
{
    u8 hw=0;
    hw|=(ADDR_0)?0x01:0x00;
    hw|=(ADDR_1)?0x02:0x00;
    hw|=(ADDR_2)?0x04:0x00;
    hw|=(ADDR_3)?0x08:0x00;
    hw|=(ADDR_4)?0x10:0x00;
    return hw+1;
}

/* ===== 寄存器读写 (应用层回调) ===== */
static unsigned short mb_read(unsigned short addr)
{
    switch(addr){case 0x0000:return 0;case 0x0001:return alarmOvercharge;
        case 0x0002:return alarmOverdischarge;case 0x0003:return curChargeMAH;
        case 0x0004:return curDischargeMAH;case 0x0005:return curBatVolt;
        case 0x0006:return curChargeCurrentMA;case 0x0007:return curDischargeCurrentMA;
        case 0x0008:return overchargeProtectVolt;case 0x0009:return overdischargeProtectVolt;
        case 0x000A:return curCycleNum;case 0x000B:return 0x0101;
        case 0x0010:return overchargeProtectVolt;case 0x0011:return overdischargeProtectVolt;
        case 0x0012:return cycleDischargeNum;case 0x0013:return chargeVoltSet;
        case 0x0014:return chargeCurrentSet;case 0x0015:return batteryType;
        default:if(addr>=0x0020&&addr<0x002A)return cycleDischargeEndMAH[addr-0x0020];
            if(addr>=0x0030&&addr<0x003A)return cycleChargeEndMAH[addr-0x0030];return 0;}
}

static unsigned char mb_write(unsigned short addr,unsigned short val)
{
    switch(addr){case 0x0000:switch(val){case 0x0001:start_discharge_req=1;break;
        case 0x0002:stop_discharge_req=1;break;case 0x0003:start_charge_req=1;break;
        case 0x0004:stop_charge_req=1;break;case 0x0005:stop_charge_req=1;stop_discharge_req=1;break;
        default:return 0;}return 1;
        case 0x0010:overchargeProtectVolt=val;return 1;case 0x0011:overdischargeProtectVolt=val;return 1;
        case 0x0012:if(val==0||val>255)return 0;cycleDischargeNum=(u8)val;return 1;
        case 0x0013:chargeVoltSet=(u8)val;CHG_VOLT_SET_1=(chargeVoltSet&1)?1:0;CHG_VOLT_SET_2=(chargeVoltSet&2)?1:0;return 1;
        case 0x0014:chargeCurrentSet=(u8)val;CHG_CURRENT_SET=1;return 1;
        case 0x0015:if(val>2)return 0;batteryType=(u8)val;SetChargeParamsByBatteryType();return 1;}
    return 0;
}

/* ===== Modbus 帧处理 (当COM1.RX_TimeOut == 0 时调用) ===== */
static void mb_process(void)
{
    u8 len=COM1.RX_Cnt,i,byte_count,fc;
    unsigned short addr,cnt,val,crc;
    if(len<4)goto done;
    crc=crc16(RX1_Buffer,len-2);
    if(crc!=(((unsigned short)RX1_Buffer[len-1]<<8)|RX1_Buffer[len-2]))goto done;
    /* 地址检查 (29 = 硬件0x1C+1) */
    if(RX1_Buffer[0]!=module_addr&&RX1_Buffer[0]!=0)goto done;

    fc=RX1_Buffer[1];
    addr=((unsigned short)RX1_Buffer[2]<<8)|RX1_Buffer[3];
    cnt=((unsigned short)RX1_Buffer[4]<<8)|RX1_Buffer[5];

    /* 广播 */
    if(RX1_Buffer[0]==0){
        if(fc==0x06&&len==8){val=((unsigned short)RX1_Buffer[4]<<8)|RX1_Buffer[5];mb_write(addr,val);}
        else if(fc==0x10&&len>=9&&RX1_Buffer[6]==cnt*2){for(i=0;i<cnt;i++){val=((unsigned short)RX1_Buffer[7+i*2]<<8)|RX1_Buffer[8+i*2];mb_write(addr+i,val);}}
        goto done;
    }

    /* 单播 */
    switch(fc){
    case 0x03:
        if(len!=8||cnt==0||cnt>30)goto ex;
        /* 构建响应: [ADDR][0x03][BYTE_CNT][DATA...][CRC] */
        RX1_Buffer[0]=module_addr;RX1_Buffer[1]=0x03;RX1_Buffer[2]=cnt*2;
        for(i=0;i<cnt;i++){val=mb_read(addr+i);RX1_Buffer[3+i*2]=val>>8;RX1_Buffer[3+i*2+1]=val&0xFF;}
        crc=crc16(RX1_Buffer,3+cnt*2);
        RX1_Buffer[3+cnt*2]=crc&0xFF;RX1_Buffer[3+cnt*2+1]=crc>>8;
        for(i=0;i<5+cnt*2;i++)TX1_write2buff(RX1_Buffer[i]);
        break;
    case 0x06:if(len!=8)goto ex;val=((unsigned short)RX1_Buffer[4]<<8)|RX1_Buffer[5];if(!mb_write(addr,val))goto ex;
        for(i=0;i<8;i++)TX1_write2buff(RX1_Buffer[i]);break;
    case 0x10:if(len<9||RX1_Buffer[6]!=cnt*2)goto ex;for(i=0;i<cnt;i++){val=((unsigned short)RX1_Buffer[7+i*2]<<8)|RX1_Buffer[8+i*2];if(!mb_write(addr+i,val))goto ex;}
        RX1_Buffer[0]=module_addr;RX1_Buffer[1]=0x10;for(i=2;i<6;i++)RX1_Buffer[i]=RX1_Buffer[i];crc=crc16(RX1_Buffer,6);RX1_Buffer[6]=crc&0xFF;RX1_Buffer[7]=crc>>8;
        for(i=0;i<8;i++)TX1_write2buff(RX1_Buffer[i]);break;
    default:goto ex;
    }
    goto done;
ex:
    RX1_Buffer[0]=module_addr;RX1_Buffer[1]=fc|0x80;RX1_Buffer[2]=0x02;crc=crc16(RX1_Buffer,3);RX1_Buffer[3]=crc&0xFF;RX1_Buffer[4]=crc>>8;
    for(i=0;i<5;i++)TX1_write2buff(RX1_Buffer[i]);
done:
    COM1.RX_Cnt=0;
}


/* ===== 电池管理状态机 ===== */
#define OVER_VOLT_CONFIRM_MS 50000
#define UNDER_VOLT_CONFIRM_MS 50000
#define CHG_COMPLETE_CONFIRM_MS 30000
#define DISCHG_COMPLETE_CONFIRM_MS 30000

typedef enum{STATE_IDLE,STATE_CHARGING,STATE_DISCHARGING,STATE_CHARGE_COMPLETE,STATE_DISCHARGE_COMPLETE}SystemState;
static SystemState current_state=STATE_IDLE;
static unsigned long charge_start_tick,discharge_start_tick;
static unsigned long chg_overc_wait_start_tick,dischg_overd_wait_start_tick;
static unsigned long chg_complete_wait_start_tick,dischg_complete_wait_start_tick;
static unsigned char chg_overc_waiting,dischg_overd_waiting;
static unsigned char chg_complete_waiting,dischg_complete_waiting;

#if EXECUTOR_DEBUG_UART
static void dbg_puts(char *s)
{
    while(*s) TX2_write2buff(*s++);
}

static void dbg_u16(unsigned int v)
{
    char buf[6];
    unsigned char i=0,j;
    if(v==0){TX2_write2buff('0');return;}
    while(v>0&&i<5){buf[i++]='0'+(v%10);v/=10;}
    for(j=0;j<i;j++) TX2_write2buff(buf[i-1-j]);
}

static void dbg_state(void)
{
    dbg_puts("DBG A=");
    dbg_u16(module_addr);
    dbg_puts(" S=");
    dbg_u16((unsigned int)current_state);
    dbg_puts(" OC=");
    dbg_u16((unsigned int)alarmOvercharge);
    dbg_puts(" OD=");
    dbg_u16((unsigned int)alarmOverdischarge);
    dbg_puts(" V=");
    dbg_u16(curBatVolt);
    dbg_puts(" IC=");
    dbg_u16(curChargeCurrentMA);
    dbg_puts(" ID=");
    dbg_u16(curDischargeCurrentMA);
    dbg_puts(" QC=");
    dbg_u16(curChargeMAH);
    dbg_puts(" QD=");
    dbg_u16(curDischargeMAH);
    dbg_puts("\r\n");
}

static void dbg_poll(void)
{
    static unsigned long last=0;
    unsigned long now=TimeStamp_GetTick();
    if((now-last)>=1000UL){last=now;dbg_state();}
}
#endif

static void bat_process(void)
{
    {static u8 d=0;if(++d>=10){d=0;Capacity_Update();}}
    if(stop_discharge_req){stop_discharge_req=0;if(current_state==STATE_DISCHARGING||current_state==STATE_DISCHARGE_COMPLETE){DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_IDLE;}}
    if(stop_charge_req){stop_charge_req=0;if(current_state==STATE_CHARGING||current_state==STATE_CHARGE_COMPLETE){CHARGE_CTRL=0;current_state=STATE_IDLE;}}
    if(start_charge_req){start_charge_req=0;if(current_state==STATE_IDLE){SetChargeParamsByBatteryType();CHARGE_CTRL=1;CHG_CURRENT_SET=1;current_state=STATE_CHARGING;charge_start_tick=TimeStamp_GetTick();}}
    if(start_discharge_req){start_discharge_req=0;if(current_state==STATE_IDLE){FAN_CTRL=1;DISCHARGE_CTRL=1;current_state=STATE_DISCHARGING;discharge_start_tick=TimeStamp_GetTick();}}
    Capacity_Update();
    switch(current_state){
    case STATE_IDLE:CHG_LED=0;DISCHG_LED=0;chg_overc_waiting=0;dischg_overd_waiting=0;curCycleNum=0;break;
    case STATE_CHARGING:CHG_LED=1;DISCHG_LED=0;if((TimeStamp_GetTick()-charge_start_tick)<50000)break;if(curBatVolt>=overchargeProtectVolt&&curChargeCurrentMA>300){if(!chg_overc_waiting){chg_overc_waiting=1;chg_overc_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-chg_overc_wait_start_tick)>=OVER_VOLT_CONFIRM_MS){CHARGE_CTRL=0;current_state=STATE_IDLE;chg_overc_waiting=0;}}else chg_overc_waiting=0;if(curChargeCurrentMA<=300){if(!chg_complete_waiting){chg_complete_waiting=1;chg_complete_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-chg_complete_wait_start_tick)>=CHG_COMPLETE_CONFIRM_MS){CHARGE_CTRL=0;current_state=STATE_CHARGE_COMPLETE;chg_complete_waiting=0;}}else chg_complete_waiting=0;break;
    case STATE_DISCHARGING:CHG_LED=0;DISCHG_LED=1;if((TimeStamp_GetTick()-discharge_start_tick)<50000)break;if(curBatVolt<=overdischargeProtectVolt&&curDischargeCurrentMA>300){if(!dischg_overd_waiting){dischg_overd_waiting=1;dischg_overd_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-dischg_overd_wait_start_tick)>=UNDER_VOLT_CONFIRM_MS){DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_IDLE;dischg_overd_waiting=0;}}else dischg_overd_waiting=0;if(curDischargeCurrentMA<=300){if(!dischg_complete_waiting){dischg_complete_waiting=1;dischg_complete_wait_start_tick=TimeStamp_GetTick();}else if((TimeStamp_GetTick()-dischg_complete_wait_start_tick)>=DISCHG_COMPLETE_CONFIRM_MS){DISCHARGE_CTRL=0;FAN_CTRL=0;current_state=STATE_DISCHARGE_COMPLETE;dischg_complete_waiting=0;}}else dischg_complete_waiting=0;break;
    case STATE_CHARGE_COMPLETE:CHG_LED=0;DISCHG_LED=0;break;
    case STATE_DISCHARGE_COMPLETE:CHG_LED=0;DISCHG_LED=0;break;}
}


/* ===== Main (基于 STC 例程模板) ===== */
void I2C_config(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    I2C_SW(I2C_P14_P15);
    I2C_InitStructure.I2C_Mode=I2C_Mode_Slave;
    I2C_InitStructure.I2C_Enable=ENABLE;
    I2C_InitStructure.I2C_Speed=60;
    I2C_InitStructure.I2C_MS_WDTA=DISABLE;
    I2C_InitStructure.I2C_SL_ADR=I2C_SLAVE_ADDR;
    I2C_InitStructure.I2C_SL_MA=ENABLE;
    I2C_Init(&I2C_InitStructure);
    NVIC_I2C_Init(I2C_Mode_Slave,I2C_ESTAI|I2C_ERXI|I2C_ETXI|I2C_ESTOI,Priority_1);
}

void UART_config(void)
{
    COMx_InitDefine COMx_InitStructure;
    COMx_InitStructure.UART_Mode=UART_8bit_BRTx;
    COMx_InitStructure.UART_BRT_Use=BRT_Timer2;
    COMx_InitStructure.UART_BaudRate=256000ul;
    COMx_InitStructure.UART_RxEnable=ENABLE;
    COMx_InitStructure.BaudRateDouble=DISABLE;
    UART_Configuration(UART1,&COMx_InitStructure);
    NVIC_UART1_Init(ENABLE,Priority_1);
    UART1_SW(UART1_SW_P30_P31);
}

#if EXECUTOR_DEBUG_UART
void DebugUART_config(void)
{
    COMx_InitDefine COMx_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin=GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.Mode=GPIO_PullUp;
    GPIO_Inilize(GPIO_P4,&GPIO_InitStructure);
    UART2_SW(UART2_SW_P46_P47);
    COMx_InitStructure.UART_Mode=UART_8bit_BRTx;
    COMx_InitStructure.UART_BRT_Use=BRT_Timer2;
    COMx_InitStructure.UART_BaudRate=256000ul;
    COMx_InitStructure.UART_RxEnable=DISABLE;
    COMx_InitStructure.BaudRateDouble=DISABLE;
    UART_Configuration(UART2,&COMx_InitStructure);
    NVIC_UART2_Init(ENABLE,Priority_0);
}
#endif

void XOSC_Init(void){P_SW2|=0x80;XOSCCR=0xC0;while(!(XOSCCR&0x01));CLKDIV=0x00;CLKSEL=0x01;}

void SetChargeParamsByBatteryType(void){switch(batteryType){case 0:CHG_VOLT_SET_1=0;CHG_VOLT_SET_2=0;chargeCurrentSet=100;overchargeProtectVolt=21500;overdischargeProtectVolt=12000;break;case 1:CHG_VOLT_SET_1=1;CHG_VOLT_SET_2=1;chargeCurrentSet=50;overchargeProtectVolt=17500;overdischargeProtectVolt=10000;break;case 2:CHG_VOLT_SET_1=1;CHG_VOLT_SET_2=1;chargeCurrentSet=50;overchargeProtectVolt=15000;overdischargeProtectVolt=8500;break;}}
void PWM_config(void){}

void main(void)
{
    EAXSFR();XOSC_Init();
    CHARGE_CTRL=0;DISCHARGE_CTRL=0;FAN_CTRL=0;
    charge_capacity_num=0;discharge_capacity_num=0;discharge_capacity_int=0;discharge_capacity_frac=0;
    charge_capacity_int=0;charge_capacity_frac=0;curBatVolt=0;
    overchargeProtectVolt=21000;overdischargeProtectVolt=16000;
    readOverchargeProtectVolt=0;readOverdischargeProtectVolt=0;
    cycleDischargeNum=1;curCycleNum=0;alarmOvercharge=0;alarmOverdischarge=0;
    memset(cycleDischargeEndMAH,0,20);memset(cycleChargeEndMAH,0,20);
    chargeVoltSet=0;chargeCurrentSet=0;batteryType=0;charge_start_tick=0;discharge_start_tick=0;adc_bat_code=0;

    GPIO_config();
    module_addr=get_hw_addr();
    I2C_config();
    UART_config();
#if EXECUTOR_DEBUG_UART
    DebugUART_config();
#endif
    EA=1;
    printf("MBON\r\n");
    ADC_config();TimeStamp_Init();Capacity_Init();
#if EXECUTOR_DEBUG_UART
    dbg_puts("DBG BOOT\r\n");
#endif

    while(1)
    {
        delay_ms(1);  /* STC 例程标准: 1ms 延时 + 超时检测 */
        if(COM1.RX_TimeOut>0)
        {
            if(--COM1.RX_TimeOut==0)
            {
                if(COM1.RX_Cnt>0) mb_process();
                COM1.RX_Cnt=0;
            }
        }
        bat_process();  /* 电池管理 */
#if EXECUTOR_DEBUG_UART
        dbg_poll();
#endif
    }
}


