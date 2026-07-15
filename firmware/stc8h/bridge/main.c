/* main.c - STC intermediary module
 *
 * Role:
 *   PC/Python serial bus <-> this addressed intermediary <-> IIC executor MCU.
 *
 * The intermediary does not run battery ADC/capacity/control logic. It only
 * parses the serial protocol and forwards register reads/writes to the executor
 * module over IIC. See protocol.c for the bridge implementation.
 */

#include "config.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "STC8G_H_Delay.h"
#include "protocol.h"

static void Bridge_GPIO_config(void)
{
    GPIO_InitTypeDef g;

    /* Bridge address inputs:
       bit5..bit0 = P0.1, P0.0, P2.7, P2.6, P2.5, P2.4.
       The user note listed P2.6 twice; the lower duplicate is treated as P2.5. */
    g.Mode = GPIO_HighZ;
    g.Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Inilize(GPIO_P0, &g);
    g.Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Inilize(GPIO_P2, &g);

    /* UART2 P1.0/P1.1 */
    g.Mode = GPIO_PullUp;
    g.Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Inilize(GPIO_P1, &g);

    /* IIC master P1.4/P1.5 */
    g.Mode = GPIO_OUT_OD;
    g.Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Inilize(GPIO_P1, &g);
}

void UART_config(void)
{
    COMx_InitDefine s;
    s.UART_Mode=UART_8bit_BRTx;
    s.UART_BRT_Use=BRT_Timer2;
    s.UART_BaudRate=256000ul;
    s.UART_RxEnable=ENABLE;
    s.BaudRateDouble=DISABLE;
    UART_Configuration(UART2,&s);
    NVIC_UART2_Init(ENABLE,Priority_1);
    UART2_SW(UART2_SW_P10_P11);
}

void XOSC_Init(void)
{
    P_SW2|=0x80;
    XOSCCR=0xC0;
    while(!(XOSCCR&0x01));
    CLKDIV=0x00;
    CLKSEL=0x01;
}

static void print_ready(void)
{
    char *p="BRIDGE\r\n";
    while(*p) TX2_write2buff(*p++);
}

void main(void)
{
    EAXSFR();
    XOSC_Init();

    Bridge_GPIO_config();
    UART_config();
    Protocol_Init(0);
    EA=1;
    print_ready();

    while(1)
    {
        delay_ms(1);
        Protocol_Poll();
        Protocol_Background();

        if(COM2.RX_TimeOut>0)
        {
            if(--COM2.RX_TimeOut==0 && COM2.RX_Cnt>0)
            {
                Protocol_Poll();
                COM2.RX_Cnt=0;
            }
        }
    }
}
