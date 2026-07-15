/* uart.c - 8051 UART1 interrupt driver */
#include "config.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_NVIC.h"
#include "STC8G_H_Switch.h"
#include "uart.h"

#define RB_SIZE 256
static unsigned char rb[RB_SIZE];
static volatile unsigned char rb_wr, rb_rd, rb_cnt;
static volatile bit tx_busy;

void UART_Init(void)
{
    COMx_InitDefine s;
    s.UART_Mode      = UART_8bit_BRTx;
    s.UART_BRT_Use   = BRT_Timer1;
    s.UART_BaudRate  = 256000ul;
    s.UART_RxEnable  = ENABLE;
    UART_Configuration(UART1, &s);
    NVIC_UART1_Init(ENABLE, Priority_1);
    UART1_SW(UART1_SW_P30_P31);
    rb_wr = 0; rb_rd = 0; rb_cnt = 0;
    tx_busy = 0;
}

void UART1_ISR(void) interrupt 4
{
    if (RI) {
        RI = 0;
        rb[rb_wr++] = SBUF;
        if (rb_wr >= RB_SIZE) rb_wr = 0;
        if (rb_cnt < RB_SIZE) rb_cnt++;
        else { rb_rd = rb_wr; rb_cnt = RB_SIZE; }
    }
    if (TI) { TI = 0; tx_busy = 0; }
}

bit UART_GetChar(unsigned char *c)
{
    if (rb_cnt == 0) return 0;
    *c = rb[rb_rd++];
    if (rb_rd >= RB_SIZE) rb_rd = 0;
    ES = 0; rb_cnt--; ES = 1;
    return 1;
}

unsigned char UART_Available(void) { return rb_cnt; }

void UART_PutChar(unsigned char c)
{
    tx_busy = 1;
    SBUF = c;
    while (tx_busy);
}

void UART_TxBuf(unsigned char xdata *data, unsigned char len)
{
    unsigned char i;
    for (i = 0; i < len; i++) UART_PutChar(data[i]);
}
