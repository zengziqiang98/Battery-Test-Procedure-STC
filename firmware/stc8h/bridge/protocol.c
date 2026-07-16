/**
 * protocol.c - STC intermediary protocol bridge
 *
 * PC side serial frame:
 *   request  [ADDR][CMD][LEN][DATA...][XOR]
 *   response [ADDR][CMD|0x80][STATUS][LEN][DATA...][XOR]
 *
 * IIC side:
 *   this MCU is master, executor MCU is IIC slave using i2c_cmd.h commands.
 */

#include "protocol.h"
#include "STC8G_H_GPIO.h"
#include "STC8G_H_UART.h"
#include "STC8G_H_I2C.h"
#include "STC8G_H_Switch.h"
#include "STC8G_H_Delay.h"
#include "i2c_cmd.h"
#include <string.h>

#define EXECUTOR_I2C_ADDR_8_FIXED (I2C_SLAVE_ADDR << 1)
#define I2C_SPEED_100K    60
#define MAX_REG_COUNT     60
#define CACHE_FIRST_REG   REG_STATUS_CHARGE
#define CACHE_LAST_REG    REG_DEVICE_ID
#define CACHE_COUNT       (CACHE_LAST_REG - CACHE_FIRST_REG + 1)
#define CACHE_DEVICE_IDX  (REG_DEVICE_ID - CACHE_FIRST_REG)
#define CACHE_DEVICE_MASK ((unsigned short)(1u << CACHE_DEVICE_IDX))
#define CACHE_FAIL_LIMIT  3
#define CACHE_FAIL_COOLDOWN_TICKS 50
#define CACHE_SUCCESS_COOLDOWN_TICKS 5
#define CACHE_AFTER_SERIAL_COOLDOWN_TICKS 0
#define SERIAL_LED_PULSE_TICKS 3
#define SERIAL_RS485_TURNAROUND_MS 1

static unsigned char proto_addr;
static unsigned char tx_buf[128];
static unsigned short cache_regs[CACHE_COUNT];
static unsigned short cache_valid_mask;
static unsigned char cache_scan_index;
static unsigned char cache_fail_count;
static unsigned char cache_cooldown;
static unsigned char cache_online;
static unsigned char rx_led_ticks;
static unsigned char tx_led_ticks;


static unsigned char xor_bytes(unsigned char *buf, unsigned char len)
{
    unsigned char i, ck = 0;
    for (i = 0; i < len; i++) ck ^= buf[i];
    return ck;
}

static unsigned char iic_ack_ok(void)
{
    RecvACK();
    return (I2CMSST & 0x01) == 0;
}

static void mark_rx_activity(void)
{
    P03 = 0;
    rx_led_ticks = SERIAL_LED_PULSE_TICKS;
}

static void mark_tx_activity(void)
{
    P02 = 0;
    tx_led_ticks = SERIAL_LED_PULSE_TICKS;
}

static void serial_send_byte(unsigned char dat)
{
    unsigned char ie2_bak;

    ie2_bak = IE2;
    IE2 &= ~0x01;
    CLR_TI2();
    S2BUF = dat;
    while (!TI2);
    CLR_TI2();
    IE2 = ie2_bak;
}

static void send_buf(unsigned char *buf, unsigned char len)
{
    unsigned char i;
    if (len > 0) mark_tx_activity();
    if (SERIAL_RS485_TURNAROUND_MS > 0) delay_ms(SERIAL_RS485_TURNAROUND_MS);
    for (i = 0; i < len; i++) serial_send_byte(buf[i]);
}

static unsigned char read_bridge_hw_addr(void)
{
    unsigned char hw = 0;

    /* bit5..bit0 = P0.1, P0.0, P2.7, P2.6, P2.5, P2.4 */
    hw |= (P01) ? 0x20 : 0x00;
    hw |= (P00) ? 0x10 : 0x00;
    hw |= (P27) ? 0x08 : 0x00;
    hw |= (P26) ? 0x04 : 0x00;
    hw |= (P25) ? 0x02 : 0x00;
    hw |= (P24) ? 0x01 : 0x00;
    return (unsigned char)(hw + 1);
}

static unsigned char iic_read_checked(unsigned char cmd, unsigned char *out, unsigned char len)
{
    unsigned char tmp[4];
    unsigned char i, ck = 0;

    if (len > 3) return 0;
    memset(tmp, 0, sizeof(tmp));

    Start();
    SendData(EXECUTOR_I2C_ADDR_8_FIXED);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(cmd);
    if (!iic_ack_ok()) { Stop(); return 0; }
    Start();
    SendData(EXECUTOR_I2C_ADDR_8_FIXED | 1);
    if (!iic_ack_ok()) { Stop(); return 0; }
    for (i = 0; i < (unsigned char)(len + 1); i++) {
        tmp[i] = RecvData();
        if (i + 1 < (unsigned char)(len + 1)) SendACK();
    }
    SendNAK();
    Stop();

    for (i = 0; i < len; i++) ck ^= tmp[i];
    if (ck != tmp[len]) return 0;
    for (i = 0; i < len; i++) out[i] = tmp[i];
    return 1;
}

static unsigned char iic_write0(unsigned char cmd)
{
    Start();
    SendData(EXECUTOR_I2C_ADDR_8_FIXED);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(cmd);
    if (!iic_ack_ok()) { Stop(); return 0; }
    Stop();
    return 1;
}

static unsigned char iic_write1(unsigned char cmd, unsigned char val)
{
    Start();
    SendData(EXECUTOR_I2C_ADDR_8_FIXED);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(cmd);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(val);
    if (!iic_ack_ok()) { Stop(); return 0; }
    Stop();
    return 1;
}

static unsigned char iic_write2(unsigned char cmd, unsigned short val)
{
    unsigned char b[2];
    b[0] = (unsigned char)(val >> 8);
    b[1] = (unsigned char)(val & 0xFF);
    Start();
    SendData(EXECUTOR_I2C_ADDR_8_FIXED);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(cmd);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(b[0]);
    if (!iic_ack_ok()) { Stop(); return 0; }
    SendData(b[1]);
    if (!iic_ack_ok()) { Stop(); return 0; }
    Stop();
    return 1;
}

static unsigned char iic_read_u8(unsigned char cmd, unsigned short *val)
{
    unsigned char b[1];
    if (!iic_read_checked(cmd, b, 1)) return 0;
    *val = b[0];
    return 1;
}

static unsigned char iic_read_u16(unsigned char cmd, unsigned short *val)
{
    unsigned char b[2];
    if (!iic_read_checked(cmd, b, 2)) return 0;
    *val = ((unsigned short)b[0] << 8) | b[1];
    return 1;
}

static unsigned char cache_index(unsigned short reg, unsigned char *idx)
{
    if (reg < CACHE_FIRST_REG || reg > CACHE_LAST_REG) return 0;
    *idx = (unsigned char)(reg - CACHE_FIRST_REG);
    return 1;
}

static void cache_store(unsigned short reg, unsigned short val)
{
    unsigned char idx;
    unsigned short mask;

    if (!cache_index(reg, &idx)) return;
    mask = (unsigned short)(1u << idx);
    cache_regs[idx] = val;
    cache_valid_mask |= mask;
}

static unsigned char cache_read(unsigned short reg, unsigned short *val)
{
    unsigned char idx;
    unsigned short mask;

    if (!cache_index(reg, &idx)) return 0;
    mask = (unsigned short)(1u << idx);
    if ((cache_valid_mask & mask) == 0) return 0;
    *val = cache_regs[idx];
    return 1;
}

static void cache_init(void)
{
    memset(cache_regs, 0, sizeof(cache_regs));
    cache_valid_mask = 0;
    cache_scan_index = 0;
    cache_fail_count = 0;
    cache_cooldown = 0;
    cache_online = 0;
    rx_led_ticks = 0;
    tx_led_ticks = 0;
    cache_store(REG_DEVICE_ID, 0x0101);
}

static unsigned char read_cached_reg_live(unsigned short reg, unsigned short *val)
{
    switch (reg) {
    case REG_STATUS_CHARGE:      return iic_read_u8(CMD_ALARM_OVERCHARGE, val);
    case REG_STATUS_DISCHARGE:   return iic_read_u8(CMD_ALARM_OVERDISCHARGE, val);
    case REG_CHARGE_MAH:         return iic_read_u16(CMD_CUR_CHARGE_MAH, val);
    case REG_DISCHARGE_MAH:      return iic_read_u16(CMD_CUR_DISCHARGE_MAH, val);
    case REG_BAT_VOLT:           return iic_read_u16(CMD_CUR_BAT_VOLT, val);
    case REG_CHARGE_CURRENT:     return iic_read_u16(CMD_CUR_CHARGE_CURRENT, val);
    case REG_DISCHARGE_CURRENT:  return iic_read_u16(CMD_CUR_DISCHARGE_CURRENT, val);
    case REG_OVERC_PROT:         return iic_read_u16(CMD_READ_OVERC_PROT_VOLT, val);
    case REG_OVERD_PROT:         return iic_read_u16(CMD_READ_OVERD_PROT_VOLT, val);
    case REG_CYCLE_NUM:          return iic_read_u8(CMD_CUR_CYCLE_NUM, val);
    case REG_DEVICE_ID:          *val = 0x0101; return 1;
    default:
        *val = 0;
        return 0;
    }
}

static unsigned char write_reg(unsigned short reg, unsigned short val)
{
    switch (reg) {
    case 0x0000:
        switch (val) {
        case PCMD_START_DISCHARGE: return iic_write0(CMD_START_DISCHARGE);
        case PCMD_STOP_DISCHARGE:  return iic_write0(CMD_STOP_DISCHARGE);
        case PCMD_START_CHARGE:    return iic_write0(CMD_START_CHARGE);
        case PCMD_STOP_CHARGE:     return iic_write0(CMD_STOP_CHARGE);
        case PCMD_EMERGENCY_STOP:
            return (unsigned char)(iic_write0(CMD_STOP_CHARGE) &
                                   iic_write0(CMD_STOP_DISCHARGE));
        default:
            return 0;
        }
    case REG_SET_OVERC_PROT:  return iic_write2(CMD_SET_OVERC_PROT_VOLT, val);
    case REG_SET_OVERD_PROT:  return iic_write2(CMD_SET_OVERD_PROT_VOLT, val);
    case REG_SET_CYCLE_NUM:   return iic_write1(CMD_SET_CYCLE_DISCH_NUM, (unsigned char)val);
    case REG_SET_CHG_VOLT:    return iic_write1(CMD_SET_CHARGE_VOLT, (unsigned char)val);
    case REG_SET_CHG_CURRENT: return iic_write1(CMD_SET_CHARGE_CURRENT, (unsigned char)val);
    case REG_SET_BAT_TYPE:    return iic_write1(CMD_SET_BAT_TYPE, (unsigned char)val);
    default:
        return 0;
    }
}

static unsigned char read_reg(unsigned short reg, unsigned short *val)
{
    unsigned char cmd;

    switch (reg) {
    case 0x0000: *val = 0; return 1;
    case REG_STATUS_CHARGE:
    case REG_STATUS_DISCHARGE:
    case REG_CHARGE_MAH:
    case REG_DISCHARGE_MAH:
    case REG_BAT_VOLT:
    case REG_CHARGE_CURRENT:
    case REG_DISCHARGE_CURRENT:
    case REG_OVERC_PROT:
    case REG_OVERD_PROT:
    case REG_CYCLE_NUM:
        return cache_read(reg, val);
    case REG_DEVICE_ID:          *val = 0x0101; return 1;
    case REG_SET_OVERC_PROT:     return cache_read(REG_OVERC_PROT, val);
    case REG_SET_OVERD_PROT:     return cache_read(REG_OVERD_PROT, val);
    case REG_SET_CYCLE_NUM:      return cache_read(REG_CYCLE_NUM, val);
    case REG_SET_CHG_VOLT:       *val = 0; return 1;
    case REG_SET_CHG_CURRENT:    *val = 0; return 1;
    case REG_SET_BAT_TYPE:       *val = 0; return 1;
    default:
        if (reg >= REG_CYCLE_DISCH_BASE && reg < REG_CYCLE_DISCH_BASE + 10) {
            cmd = CMD_CYCLE_DISCH_END_MAH_1 + (unsigned char)(reg - REG_CYCLE_DISCH_BASE);
            return iic_read_u16(cmd, val);
        }
        if (reg >= REG_CYCLE_CHG_BASE && reg < REG_CYCLE_CHG_BASE + 10) {
            cmd = CMD_CYCLE_CHARGE_END_MAH_1 + (unsigned char)(reg - REG_CYCLE_CHG_BASE);
            return iic_read_u16(cmd, val);
        }
        *val = 0;
        return 0;
    }
}

static void send_status(unsigned char cmd, unsigned char status)
{
    tx_buf[0] = proto_addr;
    tx_buf[1] = cmd | 0x80;
    tx_buf[2] = status;
    tx_buf[3] = 0;
    tx_buf[4] = xor_bytes(tx_buf, 4);
    send_buf(tx_buf, 5);
}

static void handle_frame(unsigned char *frm, unsigned char frame_len)
{
    unsigned char cmd, len, i, cnt, data_len, status;
    unsigned short reg, val;

    if (frame_len < 4) return;
    if (frm[0] != proto_addr && frm[0] != 0) return;

    len = frm[2];
    if (frame_len != (unsigned char)(4 + len)) return;
    if (xor_bytes(frm, frame_len - 1) != frm[frame_len - 1]) return;

    cmd = frm[1];

    if (frm[0] == 0) {
        if (cmd == PCMD_WRITE && len == 4) {
            reg = ((unsigned short)frm[3] << 8) | frm[4];
            val = ((unsigned short)frm[5] << 8) | frm[6];
            write_reg(reg, val);
        } else if (cmd == PCMD_START_CHARGE || cmd == PCMD_STOP_CHARGE ||
                   cmd == PCMD_START_DISCHARGE || cmd == PCMD_STOP_DISCHARGE ||
                   cmd == PCMD_EMERGENCY_STOP) {
            write_reg(0x0000, cmd);
        }
        return;
    }

    switch (cmd) {
    case PCMD_READ:
    case PCMD_BATCH_READ:
        if (len != 3) { send_status(cmd, PST_ERR); return; }
        reg = ((unsigned short)frm[3] << 8) | frm[4];
        cnt = frm[5];
        if (cnt == 0 || cnt > MAX_REG_COUNT) { send_status(cmd, PST_ERR); return; }

        tx_buf[0] = proto_addr;
        tx_buf[1] = cmd | 0x80;
        tx_buf[2] = PST_OK;
        tx_buf[3] = cnt * 2;
        data_len = tx_buf[3];
        status = PST_OK;

        for (i = 0; i < cnt; i++) {
            if (!read_reg(reg + i, &val)) {
                status = PST_ERR;
                val = 0;
            }
            tx_buf[4 + i * 2] = (unsigned char)(val >> 8);
            tx_buf[5 + i * 2] = (unsigned char)(val & 0xFF);
        }
        tx_buf[2] = status;
        tx_buf[4 + data_len] = xor_bytes(tx_buf, 4 + data_len);
        send_buf(tx_buf, 5 + data_len);
        return;

    case PCMD_WRITE:
        if (len != 4) { send_status(cmd, PST_ERR); return; }
        reg = ((unsigned short)frm[3] << 8) | frm[4];
        val = ((unsigned short)frm[5] << 8) | frm[6];
        send_status(cmd, write_reg(reg, val) ? PST_OK : PST_ERR);
        return;

    case PCMD_START_CHARGE:
    case PCMD_STOP_CHARGE:
    case PCMD_START_DISCHARGE:
    case PCMD_STOP_DISCHARGE:
    case PCMD_EMERGENCY_STOP:
        send_status(cmd, write_reg(0x0000, cmd) ? PST_OK : PST_ERR);
        return;

    default:
        send_status(cmd, PST_ERR);
        return;
    }
}

void Protocol_Init(unsigned char addr)
{
    I2C_InitTypeDef i2c;

    if (addr == 0) {
        proto_addr = read_bridge_hw_addr();
    } else {
        proto_addr = addr;
    }

    I2C_SW(I2C_P14_P15);
    i2c.I2C_Mode = I2C_Mode_Master;
    i2c.I2C_Enable = ENABLE;
    i2c.I2C_Speed = I2C_SPEED_100K;
    i2c.I2C_MS_WDTA = DISABLE;
    i2c.I2C_SL_ADR = 0;
    i2c.I2C_SL_MA = DISABLE;
    I2C_Init(&i2c);
    cache_init();
}

void Protocol_ActivityTick(void)
{
    if (rx_led_ticks > 0) {
        if (--rx_led_ticks == 0) P03 = 1;
    }
    if (tx_led_ticks > 0) {
        if (--tx_led_ticks == 0) P02 = 1;
    }
}

void Protocol_Poll(void)
{
    unsigned char len, expected;

    if (COM2.RX_Cnt > 0) mark_rx_activity();
    if (COM2.RX_Cnt < 4) return;

    len = RX2_Buffer[2];
    if (len > 124) {
        COM2.RX_Cnt = 0;
        return;
    }
    expected = 4 + len;
    if (COM2.RX_Cnt < expected) return;

    handle_frame(RX2_Buffer, expected);
    cache_cooldown = CACHE_AFTER_SERIAL_COOLDOWN_TICKS;
    COM2.RX_Cnt = 0;
}

void Protocol_Background(void)
{
    unsigned short reg;
    unsigned short val;

    if (COM2.RX_Cnt > 0) return;

    if (cache_cooldown > 0) {
        cache_cooldown--;
        return;
    }

    cache_store(REG_DEVICE_ID, 0x0101);

    if (cache_scan_index >= CACHE_DEVICE_IDX) cache_scan_index = 0;
    reg = (unsigned short)(CACHE_FIRST_REG + cache_scan_index);

    if (read_cached_reg_live(reg, &val)) {
        cache_online = 1;
        cache_store(reg, val);
        cache_fail_count = 0;
        cache_cooldown = CACHE_SUCCESS_COOLDOWN_TICKS;
    } else {
        if (cache_fail_count < 255) cache_fail_count++;
        if (cache_fail_count >= CACHE_FAIL_LIMIT) {
            cache_valid_mask &= CACHE_DEVICE_MASK;
            cache_online = 0;
        }
        cache_cooldown = CACHE_FAIL_COOLDOWN_TICKS;
    }

    cache_scan_index++;
    if (cache_scan_index >= CACHE_DEVICE_IDX) cache_scan_index = 0;
}

unsigned char Protocol_GetAddr(void)
{
    return proto_addr;
}



