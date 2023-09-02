#pragma once

#define MSG_PREFIX '#'
#define MSG_POSTFIX '$'

#define BUFFSIZE 384

#include <string.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <EEPROM.h>
#include <util/crc16.h>
#include <TMC2209.h>
#include <ArduinoJson.h>

#include "CommandBuffer.h"
#include "Motor.h"

class Settings {
  public:
    static Settings &i();
    bool runCommand(const char *cmd);
    void setup();
    void loop();
  private:
    Settings();
    ~Settings();
    Settings(const Settings&);
    Settings& operator=(const Settings&);
    void readEEPROM(uint16_t address, uint8_t *buff, uint16_t n);
    void writeEEPROM(uint16_t address, uint8_t *buff, uint16_t n);
    bool loadFromEEPROM();
    void saveToEEPROM();
    void applySettings();
    uint16_t crcCalc(uint8_t *data, uint16_t n);
    uint16_t crcCalc(const __FlashStringHelper *data, uint16_t n);
    uint16_t crcCalc(const char *str);

    bool runStatus(const char *cmd);
    bool runSetTarget(const char *cmd);
    bool runSetTeethSmall(const char *cmd);
    bool runSetTeethBig(const char *cmd);
    bool runSetRunCurrent(const char *cmd);
    bool runSetHoldCurrent(const char *cmd);
    bool runSetMotorSteps(const char *cmd);
    bool runSetMicroStepping(const char *cmd);
    bool runSetInverted(const char *cmd);
    bool runSetStandStillMode(const char *cmd);
    void runUnknownCommand();
    void saveAndAck();
    void sendMessage(char *msg);
    void sendMessage(const __FlashStringHelper *msg);
    void sendErrorMessage(char *msg);
    void sendErrorMessage(const __FlashStringHelper *msg);
    void sendStatus();

    uint16_t m_teethSmall = 20;
    uint16_t m_teethBig = 140;
    uint8_t m_runCurrent = 100;
    uint8_t m_holdCurrent = 100;
    uint16_t m_motorSteps = 200;
    uint8_t m_microStepping = 16;
    bool m_invert = false;
    TMC2209::StandstillMode m_standStillMode = TMC2209::StandstillMode::NORMAL;
};
