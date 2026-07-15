#include "BatteryTester.h"

#define I2C_DEBUG 1

// 与子模块的I²C命令码（与协议无关，按硬件定义）
#define CMD_START_DISCHARGE     0x01
#define CMD_STOP_DISCHARGE      0x02
#define CMD_START_CHARGE        0x03
#define CMD_STOP_CHARGE         0x04
#define CMD_ALARM_OVERCHARGE    0x05
#define CMD_ALARM_OVERDISCHARGE 0x06
#define CMD_SET_OVERC_PROT_VOLT 0x07
#define CMD_SET_OVERD_PROT_VOLT 0x08
#define CMD_CUR_CHARGE_MAH      0x09
#define CMD_CUR_DISCHARGE_MAH   0x0A
#define CMD_CUR_BAT_VOLT        0x0B
#define CMD_READ_OVERC_PROT_VOLT 0x0C
#define CMD_READ_OVERD_PROT_VOLT 0x0D
#define CMD_SET_CYCLE_DISCH_NUM 0x0E
#define CMD_CUR_CYCLE_NUM       0x0F
#define CMD_CYCLE_DISCH_END_MAH_1  0x10
#define CMD_CYCLE_CHARGE_END_MAH_1 0x1A
#define CMD_SET_CHARGE_VOLT        0x24
#define CMD_SET_CHARGE_CURRENT     0x25
#define CMD_READ_BAT_VOLT          0x26
#define CMD_SET_BAT_TYPE           0x27
#define CMD_CUR_CHARGE_CURRENT     0x28
#define CMD_CUR_DISCHARGE_CURRENT  0x29
#define CMD_READ_STATUS            0x2A   // 原状态寄存器，不再使用，保留兼容

// I²C通信重试次数
#define I2C_RETRY_COUNT 2

BatteryTester::BatteryTester(uint8_t addr) : _addr(addr), _wire(&Wire) {}

void BatteryTester::begin(int sda, int scl) {
    if (sda != -1 && scl != -1) {
        _wire->begin(sda, scl);
    } else {
        _wire->begin();
    }
}

bool BatteryTester::writeCmd(uint8_t cmd) {
    for (int retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
        _wire->beginTransmission(_addr);
        _wire->write(cmd);
        if (_wire->endTransmission() == 0) return true;
        delay(1);
    }
    return false;
}

bool BatteryTester::writeCmd16(uint8_t cmd, uint16_t data) {
    for (int retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
        _wire->beginTransmission(_addr);
        _wire->write(cmd);
        _wire->write((data >> 8) & 0xFF);
        _wire->write(data & 0xFF);
        if (_wire->endTransmission() == 0) return true;
        delay(1);
    }
    return false;
}

bool BatteryTester::writeCmd8(uint8_t cmd, uint8_t data) {
    for (int retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
        _wire->beginTransmission(_addr);
        _wire->write(cmd);
        _wire->write(data);
        if (_wire->endTransmission() == 0) return true;
        delay(1);
    }
    return false;
}

bool BatteryTester::readBytes(uint8_t cmd, uint8_t *buf, uint8_t len) {
    uint8_t totalLen = len + 1;  // +1 for checksum byte
    for (int retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
        _wire->beginTransmission(_addr);
        _wire->write(cmd);
        if (_wire->endTransmission(false) != 0) { delay(1); continue; }
        uint8_t received = _wire->requestFrom(_addr, totalLen);
        if (received == totalLen) {
            uint8_t ck = 0;
            for (uint8_t i = 0; i < len; i++) {
                buf[i] = _wire->read();
                ck ^= buf[i];
            }
            uint8_t rxCk = _wire->read();
            if (ck != rxCk) {
                if (I2C_DEBUG) {
                    Serial.printf("[I2C] CKSUM FAIL addr=0x%02X cmd=0x%02X calc=0x%02X rx=0x%02X retry=%d\n", _addr, cmd, ck, rxCk, retry);
                }
                delay(1);
                continue;  // checksum mismatch, retry
            }
            return true;
        }
        delay(1);
    }
    return false;
}

// ---------- 写命令实现 ----------
bool BatteryTester::startDischarge() { return writeCmd(CMD_START_DISCHARGE); }
bool BatteryTester::stopDischarge()  { return writeCmd(CMD_STOP_DISCHARGE); }
bool BatteryTester::startCharge()    { return writeCmd(CMD_START_CHARGE); }
bool BatteryTester::stopCharge()     { return writeCmd(CMD_STOP_CHARGE); }
bool BatteryTester::setOverchargeProtectVoltage(uint16_t voltage) {
    return writeCmd16(CMD_SET_OVERC_PROT_VOLT, voltage);
}
bool BatteryTester::setOverdischargeProtectVoltage(uint16_t voltage) {
    return writeCmd16(CMD_SET_OVERD_PROT_VOLT, voltage);
}
bool BatteryTester::setCycleDischargeNum(uint8_t num) {
    return writeCmd8(CMD_SET_CYCLE_DISCH_NUM, num);
}
bool BatteryTester::setChargeVoltage(uint8_t voltage) {
    return writeCmd8(CMD_SET_CHARGE_VOLT, voltage);
}
bool BatteryTester::setChargeCurrent(uint8_t current) {
    return writeCmd8(CMD_SET_CHARGE_CURRENT, current);
}
bool BatteryTester::setBatteryType(uint8_t type) {
    return writeCmd8(CMD_SET_BAT_TYPE, type);
}

// ---------- 读命令实现 ----------
bool BatteryTester::readAlarmOvercharge(bool &alarm) {
    uint8_t buf;
    if (!readBytes(CMD_ALARM_OVERCHARGE, &buf, 1)) return false;
    alarm = (buf & 0x01);     // bit0: 过充报警
    return true;
}
bool BatteryTester::readAlarmOverdischarge(bool &alarm) {
    uint8_t buf;
    if (!readBytes(CMD_ALARM_OVERDISCHARGE, &buf, 1)) return false;
    alarm = (buf & 0x01);     // bit0: 过放报警
    return true;
}
bool BatteryTester::readCurrentChargeMAH(uint16_t &mah) {
    uint8_t buf[2];
    if (!readBytes(CMD_CUR_CHARGE_MAH, buf, 2)) return false;
    mah = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readCurrentDischargeMAH(uint16_t &mah) {
    uint8_t buf[2];
    if (!readBytes(CMD_CUR_DISCHARGE_MAH, buf, 2)) return false;
    mah = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readCurrentBatteryVoltage(uint16_t &mv) {
    uint8_t buf[2];
    if (!readBytes(CMD_CUR_BAT_VOLT, buf, 2)) return false;
    mv = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readOverchargeProtectVoltage(uint16_t &mv) {
    uint8_t buf[2];
    if (!readBytes(CMD_READ_OVERC_PROT_VOLT, buf, 2)) return false;
    mv = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readOverdischargeProtectVoltage(uint16_t &mv) {
    uint8_t buf[2];
    if (!readBytes(CMD_READ_OVERD_PROT_VOLT, buf, 2)) return false;
    mv = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readCurrentCycleNum(uint8_t &num) {
    uint8_t buf;
    if (!readBytes(CMD_CUR_CYCLE_NUM, &buf, 1)) return false;
    num = buf;
    return true;
}
bool BatteryTester::readCycleDischargeEndMAH(uint8_t index, uint16_t &mah) {
    if (index < 1 || index > 10) return false;
    uint8_t cmd = CMD_CYCLE_DISCH_END_MAH_1 + (index - 1);
    uint8_t buf[2];
    if (!readBytes(cmd, buf, 2)) return false;
    mah = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readCycleChargeEndMAH(uint8_t index, uint16_t &mah) {
    if (index < 1 || index > 10) return false;
    uint8_t cmd = CMD_CYCLE_CHARGE_END_MAH_1 + (index - 1);
    uint8_t buf[2];
    if (!readBytes(cmd, buf, 2)) return false;
    mah = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readBatteryVoltage(uint16_t &mv) {
    uint8_t buf[2];
    if (!readBytes(CMD_READ_BAT_VOLT, buf, 2)) return false;
    mv = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readCurrentChargeCurrent(uint16_t &ma) {
    uint8_t buf[2];
    if (!readBytes(CMD_CUR_CHARGE_CURRENT, buf, 2)) return false;
    ma = (buf[0] << 8) | buf[1];
    return true;
}
bool BatteryTester::readCurrentDischargeCurrent(uint16_t &ma) {
    uint8_t buf[2];
    if (!readBytes(CMD_CUR_DISCHARGE_CURRENT, buf, 2)) return false;
    ma = (buf[0] << 8) | buf[1];
    return true;
}

bool BatteryTester::readAllData(ModuleData &data) {
    if (!readCurrentBatteryVoltage(data.voltage)) return false;
    if (!readCurrentChargeCurrent(data.chargeCurrent)) return false;
    if (!readCurrentDischargeCurrent(data.dischargeCurrent)) return false;
    if (!readCurrentChargeMAH(data.chargeMAH)) return false;
    if (!readCurrentDischargeMAH(data.dischargeMAH)) return false;

    uint8_t chg_status = 0, dchg_status = 0;
    if (!readBytes(CMD_ALARM_OVERCHARGE, &chg_status, 1) ||
        !readBytes(CMD_ALARM_OVERDISCHARGE, &dchg_status, 1)) {
        data.status = 0;
    } else {
        uint8_t status = 0;
        // bit2 of alarm → 充电中
        if (chg_status & 0x04) status |= 0x01;
        // bit2 of alarm → 放电中
        if (dchg_status & 0x04) status |= 0x02;
        // bit0 → 过充报警
        if (chg_status & 0x01) status |= 0x04;
        // bit0 → 过放报警
        if (dchg_status & 0x01) status |= 0x08;
        // bit1 → 充电完成
        if (chg_status & 0x02) status |= 0x10;
        // bit1 → 放电完成
        if (dchg_status & 0x02) status |= 0x20;
    // No-battery: chg_status bit3 -> status bit6
    if (chg_status & 0x08) status |= 0x40;
        data.status = status;
    data.chgStatus = chg_status;
    }

    if (!readCurrentCycleNum(data.cycleNum)) return false;
    return true;
}