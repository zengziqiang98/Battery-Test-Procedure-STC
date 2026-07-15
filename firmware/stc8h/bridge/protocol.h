/**
 * protocol.h - PC serial protocol for the STC intermediary module.
 *
 * Request:
 *   [ADDR][CMD][LEN][DATA...][XOR]
 *
 * Response:
 *   [ADDR][CMD|0x80][STATUS][LEN][DATA...][XOR]
 *
 * ADDR 1..64, ADDR 0 is broadcast.
 * XOR is calculated over every byte before the checksum byte.
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "config.h"

/* Command codes */
#define PCMD_READ            0x01  /* DATA=[REG_H][REG_L][CNT] */
#define PCMD_WRITE           0x02  /* DATA=[REG_H][REG_L][VAL_H][VAL_L] */
#define PCMD_START_CHARGE    0x03
#define PCMD_STOP_CHARGE     0x04
#define PCMD_START_DISCHARGE 0x05
#define PCMD_STOP_DISCHARGE  0x06
#define PCMD_EMERGENCY_STOP  0x07
#define PCMD_BATCH_READ      0x08  /* DATA=[START_REG_H][START_REG_L][CNT] */

/* Register addresses */
#define REG_STATUS_CHARGE     0x01
#define REG_STATUS_DISCHARGE  0x02
#define REG_CHARGE_MAH        0x03
#define REG_DISCHARGE_MAH     0x04
#define REG_BAT_VOLT          0x05
#define REG_CHARGE_CURRENT    0x06
#define REG_DISCHARGE_CURRENT 0x07
#define REG_OVERC_PROT        0x08
#define REG_OVERD_PROT        0x09
#define REG_CYCLE_NUM         0x0A
#define REG_DEVICE_ID         0x0B
#define REG_SET_OVERC_PROT    0x10
#define REG_SET_OVERD_PROT    0x11
#define REG_SET_CYCLE_NUM     0x12
#define REG_SET_CHG_VOLT      0x13
#define REG_SET_CHG_CURRENT   0x14
#define REG_SET_BAT_TYPE      0x15
#define REG_CYCLE_DISCH_BASE  0x20
#define REG_CYCLE_CHG_BASE    0x30

/* Response status */
#define PST_OK      0x00
#define PST_ERR     0xFF

/* API */
void Protocol_Init(unsigned char addr);
void Protocol_Poll(void);
void Protocol_Background(void);
unsigned char Protocol_GetAddr(void);

#endif
