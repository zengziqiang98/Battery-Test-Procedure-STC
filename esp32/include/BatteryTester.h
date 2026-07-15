#ifndef BATTERY_TESTER_H
#define BATTERY_TESTER_H

#include <Arduino.h>
#include <Wire.h>

extern bool noBatteryConfirmed[];
/**
 * @brief 电池测试模块驱动类
 * 通过I²C与子模块通信，提供充放电控制、参数设置、数据读取等功能
 */
class BatteryTester {
public:
    /**
     * @brief 构造函数
     * @param addr 7位I²C从机地址
     */
    BatteryTester(uint8_t addr = 0x60);

    /**
     * @brief 初始化I²C总线
     * @param sda SDA引脚，-1表示使用默认
     * @param scl SCL引脚，-1表示使用默认
     */
    void begin(int sda = -1, int scl = -1);

    /**
     * @brief 获取模块地址
     */
    uint8_t getAddress() const { return _addr; }

    /**
     * @brief 一次性读取所有数据
     */
    struct ModuleData {
        uint16_t voltage;          // mV
        uint16_t chargeCurrent;    // mA
        uint16_t dischargeCurrent; // mA
        uint16_t chargeMAH;        // mAh
        uint16_t dischargeMAH;     // mAh
        uint8_t status;            // bit0:充电中, bit1:放电中, bit2:过充报警, bit3:过放报警
        uint8_t chgStatus;         // raw chg_status from STC8H
        uint8_t cycleNum;
    };
    bool readAllData(ModuleData &data);

    // ---------- 写命令 ----------
    bool startDischarge();
    bool stopDischarge();
    bool startCharge();
    bool stopCharge();
    bool setOverchargeProtectVoltage(uint16_t voltage);   // mV
    bool setOverdischargeProtectVoltage(uint16_t voltage); // mV
    bool setCycleDischargeNum(uint8_t num);
    bool setChargeVoltage(uint8_t voltage);
    bool setChargeCurrent(uint8_t current);
    bool setBatteryType(uint8_t type);

    // ---------- 读命令（仅供内部使用，不对外暴露）----------
    bool readAlarmOvercharge(bool &alarm);
    bool readAlarmOverdischarge(bool &alarm);
    bool readCurrentChargeMAH(uint16_t &mah);
    bool readCurrentDischargeMAH(uint16_t &mah);
    bool readCurrentBatteryVoltage(uint16_t &mv);
    bool readOverchargeProtectVoltage(uint16_t &mv);
    bool readOverdischargeProtectVoltage(uint16_t &mv);
    bool readCurrentCycleNum(uint8_t &num);
    bool readCycleDischargeEndMAH(uint8_t index, uint16_t &mah);
    bool readCycleChargeEndMAH(uint8_t index, uint16_t &mah);
    bool readBatteryVoltage(uint16_t &mv);
    bool readCurrentChargeCurrent(uint16_t &ma);
    bool readCurrentDischargeCurrent(uint16_t &ma);

private:
    uint8_t _addr;
    TwoWire *_wire;

    bool writeCmd(uint8_t cmd);
    bool writeCmd16(uint8_t cmd, uint16_t data);
    bool writeCmd8(uint8_t cmd, uint8_t data);
    bool readBytes(uint8_t cmd, uint8_t *buf, uint8_t len);
};

#endif