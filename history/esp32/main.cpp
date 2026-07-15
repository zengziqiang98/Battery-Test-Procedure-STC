#include <Arduino.h>
#include <Wire.h>
#include "BatteryTester.h"

#define MODULE_COUNT 32
#define BUZZER_PIN 15

const uint8_t MODULE_ADDR[MODULE_COUNT] = {
    0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,
    0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,
    0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,
    0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F
};

#define FRAME_HEADER 0xAA
#define CMD_BATCH_READ 0x10

#define LOW_VOLTAGE_THRESHOLD     3000   // mV
#define ACTIVATE_CHARGE_DURATION  50     // 短暂充电时长 (ms)
#define ACTIVATE_WAIT_AFTER_CHARGE 1000  // 充电后等待时间 (ms)
#define ACTIVATE_CHECK_INTERVAL   5000   // 整体检查周期 (ms)
#define NO_BATTERY_RETRY_INTERVAL 30000  // 确认无电池后重试间隔 (ms)

// 单模块命令 (保留)
const uint8_t CMD_START_DISCHARGE   = 0x01;
const uint8_t CMD_STOP_DISCHARGE    = 0x02;
const uint8_t CMD_START_CHARGE      = 0x03;
const uint8_t CMD_STOP_CHARGE       = 0x04;
const uint8_t CMD_SET_OVERC_PROT    = 0x05;
const uint8_t CMD_SET_OVERD_PROT    = 0x06;
const uint8_t CMD_SET_CYCLE_NUM     = 0x07;
const uint8_t CMD_SET_CHARGE_VOLT   = 0x08;
const uint8_t CMD_SET_CHARGE_CUR    = 0x09;
const uint8_t CMD_SET_BAT_TYPE      = 0x0A;
const uint8_t CMD_BUZZER_ON         = 0x0B;
const uint8_t CMD_BUZZER_OFF        = 0x0C;

// 新增：批量/广播命令
const uint8_t CMD_ALL_START_CHARGE    = 0x20;
const uint8_t CMD_ALL_STOP_CHARGE     = 0x21;
const uint8_t CMD_ALL_START_DISCHARGE = 0x22;
const uint8_t CMD_ALL_STOP_DISCHARGE  = 0x23;
const uint8_t CMD_ALL_SET_CYCLE_NUM   = 0x24;
const uint8_t CMD_ALL_SET_BAT_TYPE    = 0x25;
const uint8_t CMD_ALL_EMERGENCY_STOP  = 0x26;
const uint8_t CMD_ALL_BUZZER_ON       = 0x27;
const uint8_t CMD_ALL_BUZZER_OFF      = 0x28;

#define SERIAL_TIMEOUT_MS 100

BatteryTester testers[MODULE_COUNT];
static uint8_t cachedData[MODULE_COUNT * 12];
static unsigned long lastCacheUpdate = 0;
const unsigned long CACHE_UPDATE_INTERVAL_MS = 1000;

static uint8_t sendBuffer[390];

// 函数声明
bool findModuleIndex(uint8_t addr, int &index);
void sendResponse(uint8_t cmd, uint8_t status, const uint8_t *data, uint16_t dataLen);
void handleCommand();
void updateAllModuleCache();
void executeAllStartCharge();
void executeAllStopCharge();
void executeAllStartDischarge();
void executeAllStopDischarge();
void executeAllSetCycleNum(uint8_t num);
void executeAllSetBatType(uint8_t type);
void executeEmergencyStop();
void executeAllBuzzerOn();
void executeAllBuzzerOff();
void checkLowVoltageModules();
// 解析状态机
enum ParseState { STATE_HEADER, STATE_CMD, STATE_ADDR, STATE_LEN_H, STATE_LEN_L, STATE_DATA, STATE_CHECKSUM };
static struct {
    ParseState state = STATE_HEADER;
    uint8_t cmd, addr;
    uint16_t len;
    uint8_t data[256];
    uint8_t dataIdx;
    uint8_t checksum;
    unsigned long lastByteTime;
} frame;

// 每个模块的激活状态
enum class ActivationState : uint8_t {
    IDLE,
    CHARGING,
    WAITING
};

static bool buzzerActive = false;
static unsigned long buzzerOnTime = 0;

static ActivationState activationState[MODULE_COUNT];
static unsigned long activationStartTime[MODULE_COUNT];
bool noBatteryConfirmed[MODULE_COUNT];
static unsigned long noBatteryTimestamp[MODULE_COUNT]; // 记录确认为无电池的时间

void setup() {
    Serial.begin(115200);
    #ifdef ESP32
        esp_log_level_set("*", ESP_LOG_NONE);
        Serial.setDebugOutput(false);
    #endif

    Wire.begin(33, 25);
    Wire.setClock(20000);
    Wire.setTimeOut(10);   // 降低 I2C 超时，避免单模块拖慢整体

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    for (int i = 0; i < MODULE_COUNT; i++) {
        testers[i] = BatteryTester(MODULE_ADDR[i]);
        testers[i].begin();
    }
    for (int i = 0; i < MODULE_COUNT; i++) {
        activationState[i] = ActivationState::IDLE;
        noBatteryConfirmed[i] = false;
        noBatteryTimestamp[i] = 0;
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    updateAllModuleCache();
    lastCacheUpdate = millis();
    Serial.println("Battery Tester Ready");
}

void loop() {
if (millis() - lastCacheUpdate >= CACHE_UPDATE_INTERVAL_MS) {
        updateAllModuleCache();
        lastCacheUpdate = millis();
    }

    if (buzzerActive && (millis() - buzzerOnTime >= 500)) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActive = false;
    }
    static unsigned long lastActivationCheck = 0;
    if (millis() - lastActivationCheck >= ACTIVATE_CHECK_INTERVAL) {
        lastActivationCheck = millis();
        checkLowVoltageModules();
    }

    while (Serial.available()) {
        uint8_t c = Serial.read();
        frame.lastByteTime = millis();
        switch (frame.state) {
            case STATE_HEADER:
                if (c == FRAME_HEADER) { frame.state = STATE_CMD; frame.checksum = 0; }
                break;
            case STATE_CMD:
                frame.cmd = c; frame.checksum ^= c; frame.state = STATE_ADDR; break;
            case STATE_ADDR:
                frame.addr = c; frame.checksum ^= c; frame.state = STATE_LEN_H; break;
            case STATE_LEN_H:
                frame.len = c << 8; frame.checksum ^= c; frame.state = STATE_LEN_L; break;
            case STATE_LEN_L:
                frame.len |= c; frame.checksum ^= c;
                if (frame.len > 256) frame.state = STATE_HEADER;
                else {
                    frame.dataIdx = 0;
                    frame.state = (frame.len == 0) ? STATE_CHECKSUM : STATE_DATA;
                }
                break;
            case STATE_DATA:
                frame.data[frame.dataIdx++] = c; frame.checksum ^= c;
                if (frame.dataIdx >= frame.len) frame.state = STATE_CHECKSUM;
                break;
            case STATE_CHECKSUM:
                if (c == frame.checksum) handleCommand();
                frame.state = STATE_HEADER;
                break;
        }
    }

    if (frame.state != STATE_HEADER && (millis() - frame.lastByteTime > SERIAL_TIMEOUT_MS)) {
        frame.state = STATE_HEADER;
    }

    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
    }
}

void handleCommand() {
    // 广播命令
    switch (frame.cmd) {
        case CMD_ALL_START_CHARGE:    executeAllStartCharge();    sendResponse(frame.cmd, 0x00, nullptr, 0); return;
        case CMD_ALL_STOP_CHARGE:     executeAllStopCharge();     sendResponse(frame.cmd, 0x00, nullptr, 0); return;
        case CMD_ALL_START_DISCHARGE: executeAllStartDischarge(); sendResponse(frame.cmd, 0x00, nullptr, 0); return;
        case CMD_ALL_STOP_DISCHARGE:  executeAllStopDischarge();  sendResponse(frame.cmd, 0x00, nullptr, 0); return;
        case CMD_ALL_SET_CYCLE_NUM:
            if (frame.len == 1) { executeAllSetCycleNum(frame.data[0]); sendResponse(frame.cmd, 0x00, nullptr, 0); }
            else sendResponse(frame.cmd, 0x01, nullptr, 0);
            return;
        case CMD_ALL_SET_BAT_TYPE:
            if (frame.len == 1) { executeAllSetBatType(frame.data[0]); sendResponse(frame.cmd, 0x00, nullptr, 0); }
            else sendResponse(frame.cmd, 0x01, nullptr, 0);
            return;
        case CMD_ALL_EMERGENCY_STOP:  executeEmergencyStop();      sendResponse(frame.cmd, 0x00, nullptr, 0); return;
        case CMD_ALL_BUZZER_ON:       executeAllBuzzerOn();        sendResponse(frame.cmd, 0x00, nullptr, 0); return;
        case CMD_ALL_BUZZER_OFF:      executeAllBuzzerOff();       sendResponse(frame.cmd, 0x00, nullptr, 0); return;
    }

    // 单模块/批量读取命令
    int moduleIndex;
    bool success = false;
    uint8_t responseData[MODULE_COUNT * 12];
    uint16_t responseLen = 0;

    if (frame.cmd == CMD_BATCH_READ) {
        responseLen = MODULE_COUNT * 12;
        memcpy(responseData, cachedData, responseLen);
        success = true;
    } else if (findModuleIndex(frame.addr, moduleIndex)) {
        switch (frame.cmd) {
            case CMD_START_DISCHARGE:
                if (testers[moduleIndex].startDischarge()) {
                    success = true;
                    activationState[moduleIndex] = ActivationState::IDLE;
                    noBatteryConfirmed[moduleIndex] = false;
                }
                break;
            case CMD_STOP_DISCHARGE:
                success = testers[moduleIndex].stopDischarge();
                activationState[moduleIndex] = ActivationState::IDLE;
                break;
            case CMD_START_CHARGE:
                if (testers[moduleIndex].startCharge()) {
                    success = true;
                    activationState[moduleIndex] = ActivationState::IDLE;
                    noBatteryConfirmed[moduleIndex] = false;
                }
                break;
            case CMD_STOP_CHARGE:
                success = testers[moduleIndex].stopCharge();
                activationState[moduleIndex] = ActivationState::IDLE;
                break;
            case CMD_BUZZER_ON:
                digitalWrite(BUZZER_PIN, HIGH); buzzerOnTime = millis(); buzzerActive = true; success = true; break;
            case CMD_BUZZER_OFF:
                digitalWrite(BUZZER_PIN, LOW); buzzerActive = false; success = true; break;
            case CMD_SET_OVERC_PROT:
                if (frame.len == 2) success = testers[moduleIndex].setOverchargeProtectVoltage((frame.data[0]<<8)|frame.data[1]);
                break;
            case CMD_SET_OVERD_PROT:
                if (frame.len == 2) success = testers[moduleIndex].setOverdischargeProtectVoltage((frame.data[0]<<8)|frame.data[1]);
                break;
            case CMD_SET_CYCLE_NUM:
                if (frame.len == 1) success = testers[moduleIndex].setCycleDischargeNum(frame.data[0]); break;
            case CMD_SET_CHARGE_VOLT:
                if (frame.len == 1) success = testers[moduleIndex].setChargeVoltage(frame.data[0]); break;
            case CMD_SET_CHARGE_CUR:
                if (frame.len == 1) success = testers[moduleIndex].setChargeCurrent(frame.data[0]); break;
            case CMD_SET_BAT_TYPE:
                if (frame.len == 1) success = testers[moduleIndex].setBatteryType(frame.data[0]); break;
        }
        // 更新该模块缓存
        if (success) {
            BatteryTester::ModuleData md;
            if (testers[moduleIndex].readAllData(md)) {
                uint8_t *ptr = cachedData + moduleIndex * 12;
                *ptr++ = (md.voltage>>8)&0xFF; *ptr++ = md.voltage&0xFF;
                *ptr++ = (md.chargeCurrent>>8)&0xFF; *ptr++ = md.chargeCurrent&0xFF;
                *ptr++ = (md.dischargeCurrent>>8)&0xFF; *ptr++ = md.dischargeCurrent&0xFF;
                *ptr++ = (md.chargeMAH>>8)&0xFF; *ptr++ = md.chargeMAH&0xFF;
                *ptr++ = (md.dischargeMAH>>8)&0xFF; *ptr++ = md.dischargeMAH&0xFF;
                *ptr++ = md.status; *ptr++ = md.cycleNum;
            }
        }
    }
    sendResponse(frame.cmd, success ? 0x00 : 0x01, responseData, responseLen);
}

// ---------- 批量操作实现 ----------
void executeAllStartCharge() {
    int failCount = 0;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (!testers[i].startCharge()) {
            failCount++;
            Serial.printf("[BCAST] START_CHARGE I2C FAIL addr=0x%02X\n", MODULE_ADDR[i]);
        }
    }
    if (failCount) Serial.printf("[BCAST] START_CHARGE: %d module(s) I2C write FAILED\n", failCount);
}
void executeAllStopCharge() {
    int failCount = 0;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (!testers[i].stopCharge()) {
            failCount++;
            Serial.printf("[BCAST] STOP_CHARGE I2C FAIL addr=0x%02X\n", MODULE_ADDR[i]);
        }
    }
    if (failCount) Serial.printf("[BCAST] STOP_CHARGE: %d module(s) I2C write FAILED\n", failCount);
}
void executeAllStartDischarge() {
    int failCount = 0;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (!testers[i].startDischarge()) {
            failCount++;
            Serial.printf("[BCAST] START_DISCHARGE I2C FAIL addr=0x%02X\n", MODULE_ADDR[i]);
        }
    }
    if (failCount) Serial.printf("[BCAST] START_DISCHARGE: %d module(s) I2C write FAILED\n", failCount);
}
void executeAllStopDischarge() {
    int failCount = 0;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (!testers[i].stopDischarge()) {
            failCount++;
            Serial.printf("[BCAST] STOP_DISCHARGE I2C FAIL addr=0x%02X\n", MODULE_ADDR[i]);
        }
    }
    if (failCount) Serial.printf("[BCAST] STOP_DISCHARGE: %d module(s) I2C write FAILED\n", failCount);
}
void executeAllSetCycleNum(uint8_t num) {
    for (int i = 0; i < MODULE_COUNT; i++) testers[i].setCycleDischargeNum(num);
}
void executeAllSetBatType(uint8_t type) {
    for (int i = 0; i < MODULE_COUNT; i++) testers[i].setBatteryType(type);
}
void executeEmergencyStop() {
    int failCount = 0;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (!testers[i].stopCharge()) failCount++;
        if (!testers[i].stopDischarge()) failCount++;
    }
    if (failCount) Serial.printf("[BCAST] EMERGENCY_STOP: %d I2C write(s) FAILED\n", failCount);
    // Clear all activation states
    for (int i = 0; i < MODULE_COUNT; i++) {
        activationState[i] = ActivationState::IDLE;
    }
}
void executeAllBuzzerOn() {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOnTime = millis();
    buzzerActive = true;
}
void executeAllBuzzerOff() {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
}

void updateAllModuleCache() {
    uint8_t *ptr = cachedData;
    
    for (int i = 0; i < MODULE_COUNT; i++) {
        BatteryTester::ModuleData md;
        if (testers[i].readAllData(md)) {
 //           uint8_t status = md.status;
 
 //           if (noBatteryConfirmed[i]) status |= 0x10;   // bit4表示无电池
            *ptr++ = (md.voltage>>8)&0xFF; *ptr++ = md.voltage&0xFF;
            *ptr++ = (md.chargeCurrent>>8)&0xFF; *ptr++ = md.chargeCurrent&0xFF;
            *ptr++ = (md.dischargeCurrent>>8)&0xFF; *ptr++ = md.dischargeCurrent&0xFF;
            *ptr++ = (md.chargeMAH>>8)&0xFF; *ptr++ = md.chargeMAH&0xFF;
            *ptr++ = (md.dischargeMAH>>8)&0xFF; *ptr++ = md.dischargeMAH&0xFF;
                        // No-battery: STC8H chg_status bit3 -> status bit6
            if (md.chgStatus & 0x08) md.status |= 0x40;
            *ptr++ = md.status; *ptr++ = md.cycleNum;
        } else {
            memset(ptr, 0, 12); ptr += 12;
        }
    }
}

bool findModuleIndex(uint8_t addr, int &index) {
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (testers[i].getAddress() == addr) { index = i; return true; }
    }
    return false;
}

void sendResponse(uint8_t cmd, uint8_t status, const uint8_t *data, uint16_t dataLen) {
    uint8_t *resp = sendBuffer;
    resp[0] = FRAME_HEADER; resp[1] = cmd; resp[2] = status;
    resp[3] = (dataLen>>8)&0xFF; resp[4] = dataLen&0xFF;
    if (dataLen) memcpy(&resp[5], data, dataLen);
    uint8_t checksum = cmd ^ status ^ resp[3] ^ resp[4];
    for (uint16_t i=0; i<dataLen; i++) checksum ^= data[i];
    resp[5+dataLen] = checksum;
    Serial.write(resp, 6+dataLen);
}


void checkLowVoltageModules() {
    unsigned long now = millis();
    for (int i = 0; i < MODULE_COUNT; i++) {
        // 如果模块当前正在充/放电（由正常命令控制），跳过自动检测
        BatteryTester::ModuleData md;
        if (!testers[i].readAllData(md)) continue; // 读失败则跳过
        bool isCharging = md.status & 0x01;
        bool isDischarging = md.status & 0x02;
        if (isCharging || isDischarging) {
            // 模块正忙，重置激活状态（可能被外部命令控制）
            activationState[i] = ActivationState::IDLE;
            continue;
        }

        // 处理不同激活状态
        switch (activationState[i]) {
            case ActivationState::IDLE: {
                // 如果电压低于阈值，启动激活流程
                if (md.voltage < LOW_VOLTAGE_THRESHOLD) {
                    // 如果已经完全确认无电池，检查是否到了重试时间
                    if (noBatteryConfirmed[i]) {
                        if (now - noBatteryTimestamp[i] >= NO_BATTERY_RETRY_INTERVAL) {
                            // 重置，重新尝试激活
                            noBatteryConfirmed[i] = false;
                            noBatteryTimestamp[i] = 0;
                            // 立即进入激活
                            testers[i].startCharge();
                            activationStartTime[i] = now;
                            activationState[i] = ActivationState::CHARGING;
                        }
                    } else {
                        // 第一次低电压，启动短暂充电
                        testers[i].startCharge();
                        activationStartTime[i] = now;
                        activationState[i] = ActivationState::CHARGING;
                    }
                }
                break;
            }
            case ActivationState::CHARGING: {
                if (now - activationStartTime[i] >= ACTIVATE_CHARGE_DURATION) {
                    testers[i].stopCharge();
                    activationStartTime[i] = now; // 记录停止时刻
                    activationState[i] = ActivationState::WAITING;
                }
                break;
            }
            case ActivationState::WAITING: {
                if (now - activationStartTime[i] >= ACTIVATE_WAIT_AFTER_CHARGE) {
                    // 读取电压
                    uint16_t volt = 0;
                    testers[i].readCurrentBatteryVoltage(volt);
                    if (volt < LOW_VOLTAGE_THRESHOLD) {
                        noBatteryConfirmed[i] = true;
                        noBatteryTimestamp[i] = now; // 记录确认时间
                    } else {
                        noBatteryConfirmed[i] = false; // 电压恢复
                    }
                    activationState[i] = ActivationState::IDLE;
                }
                break;
            }
        }
    }
}