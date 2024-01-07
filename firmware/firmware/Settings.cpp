/*
 *  This file is part of the Pollux Rotator software
 *
 *  Created by Philipp Weber
 *  Copyright (c) 2023 Philipp Weber
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */



#include "Settings.h"

#define OFFSET_MAGIC 0
static uint16_t __magic = 0b1110110101010010;

#define OFFSET_CRC (OFFSET_MAGIC + sizeof(__magic))

#define OFFSET_DATA (OFFSET_CRC + sizeof(uint16_t))

Settings &Settings::i() {
  static Settings theInstance;
  return theInstance;
}

void Settings::readEEPROM(uint16_t address, uint8_t *buff, uint16_t n) {
  for (int ii=0; ii<n; ii++) {
    buff[ii] = EEPROM.read(address+ii);
  }
}

void Settings::writeEEPROM(uint16_t address, uint8_t *buff, uint16_t n) {
  for (int ii=0; ii<n; ii++) {
    EEPROM.write(address+ii, buff[ii]);
  }
}

uint16_t Settings::crcCalc(uint8_t *data, uint16_t n) {
  uint16_t crc = 0;
  for (uint16_t ii=0; ii<n; ii++) {
    crc = _crc16_update(crc, data[ii]);
  }
  return crc;
}

uint16_t Settings::crcCalc(const char *str) {
  uint16_t crc = 0;
  while ( *str != '\0' ) {
    crc = _crc16_update(crc, *str);
    str++;
  }
  return crc;
}

uint16_t Settings::crcCalc(const __FlashStringHelper *data, uint16_t n) {
  const char *ptr = reinterpret_cast<const char *>(data);
  uint16_t crc = 0;
  for (uint16_t ii=0; ii<n; ii++) {
    crc = _crc16_update(crc, pgm_read_byte(ptr+ii));
  }
  return crc;
}

bool Settings::loadFromEEPROM() {
  uint16_t magic;
  uint16_t crc;
  uint8_t data[sizeof(Settings)];

  readEEPROM(OFFSET_MAGIC, (uint8_t *) &magic, sizeof(magic));
  if ( magic != __magic ) {
    return false;
  }

  readEEPROM(OFFSET_CRC, (uint8_t *) &crc, sizeof(crc));
  readEEPROM(OFFSET_DATA, data, sizeof(Settings));

  if ( crc != crcCalc(data, sizeof(Settings)) ) {
    return false;
  }

  memcpy((void *) this, (void *) data, sizeof(Settings));
  return true;
}

void Settings::saveToEEPROM() {
  uint16_t crc = crcCalc((uint8_t *) this, sizeof(Settings));
  writeEEPROM(OFFSET_DATA, (uint8_t *) this, sizeof(Settings));
  writeEEPROM(OFFSET_CRC, (uint8_t *) &crc, sizeof(crc));
  writeEEPROM(OFFSET_MAGIC, (uint8_t *) &__magic, sizeof(__magic));
}

void Settings::applySettings() {
	Motor::i().setGear(m_teethSmall, m_teethBig, m_motorSteps, m_microStepping);
    Motor::i().setSpeed(m_speed);
    Motor::i().setRunCurrent(m_runCurrent);
    Motor::i().setHoldCurrent(m_holdCurrent);
    Motor::i().setInverted(m_invert);
    Motor::i().setStandStillMode(m_standStillMode);
    Motor::i().setStealthChop(m_stealthChop);
    Motor::i().setCoolStep(m_coolStep);
}

Settings::Settings() {
  if ( ! loadFromEEPROM() ) {
    saveToEEPROM();
  }
  applySettings();
}

Settings::~Settings() {
}

void Settings::sendMessage(char *msg) {
  uint16_t crc = crcCalc(msg);
  Motor::i().update();
  char crcMsg[3];
  crcMsg[0] = ((char *) &crc)[0];
  crcMsg[1] = ((char *) &crc)[1];
  crcMsg[2] = '\0';
  // Must handle '$' which is end of message character
  if ( crcMsg[0] == '$' ) {
    crcMsg[0] = '1';
  }
  if ( crcMsg[1] == '$' ) {
    crcMsg[1] = '1';
  }
  Serial.print(MSG_PREFIX);
  Motor::i().update();
  Serial.print(msg);
  Motor::i().update();
  Serial.print('\0');
  Motor::i().update();
  Serial.print(crcMsg[0]);
  Motor::i().update();
  Serial.print(crcMsg[1]);
  Motor::i().update();
  Serial.print(MSG_POSTFIX);
  Motor::i().update();
}

void Settings::sendMessage(const __FlashStringHelper *msg) {
  uint16_t crc = crcCalc(msg, strlen_P(msg));
  Motor::i().update();
  char crcMsg[3];
  crcMsg[0] = ((char *) &crc)[0];
  crcMsg[1] = ((char *) &crc)[1];
  crcMsg[2] = '\0';
  // Must handle '$' which is end of message character
  if ( crcMsg[0] == '$' ) {
    crcMsg[0] = '1';
  }
  if ( crcMsg[1] == '$' ) {
    crcMsg[1] = '1';
  }
  Serial.print(MSG_PREFIX);
  Motor::i().update();
  Serial.print(msg);
  Motor::i().update();
  Serial.print('\0');
  Motor::i().update();
  Serial.print(crcMsg[0]);
  Motor::i().update();
  Serial.print(crcMsg[1]);
  Motor::i().update();
  Serial.print(MSG_POSTFIX);
  Motor::i().update();
}

void Settings::sendErrorMessage(char *msg) {
  char buff[128];
  DynamicJsonDocument json(128);
  json[F("Error")] = msg;
  serializeJson(json, buff, 128);
  sendMessage(buff);
}

void Settings::sendErrorMessage(const __FlashStringHelper *msg) {
  char buff[128];
  DynamicJsonDocument json(128);
  json[F("Error")] = msg;
  serializeJson(json, buff, 128);
  sendMessage(buff);
}

void Settings::sendStatus() {
  char buff[BUFFSIZE];
  Motor::i().update();
  Motor::i().state(buff, BUFFSIZE);
  Motor::i().update();
  sendMessage(buff);
  Motor::i().update();
}

bool Settings::runStatus(const char *cmd) {
  if ( strcmp_P(cmd, F("status") ) ) {
    return false;
  }
  Motor::i().update();
  sendStatus();
  Motor::i().update();
  return true;
}

bool Settings::runSetTarget(const char *cmd) {
    uint16_t target;
    if ( sscanf_P(cmd, PSTR("set target %u"), &target) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setTargetAngle((float) target / 100.);
    Motor::i().update();
    sendStatus();
    Motor::i().update();
    return true;
}

bool Settings::runSetTeethSmall(const char *cmd) {
    uint16_t teeth;
    if ( sscanf_P(cmd, PSTR("set teeth small %u"), &teeth) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setTeethSmall(teeth);
    Motor::i().update();
    m_teethSmall = teeth;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetTeethBig(const char *cmd) {
    uint16_t teeth;
    if ( sscanf_P(cmd, PSTR("set teeth big %u"), &teeth) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setTeethBig(teeth);
    Motor::i().update();
    m_teethBig = teeth;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetRunCurrent(const char *cmd) {
    uint8_t current;
    if ( sscanf_P(cmd, PSTR("set rc %u"), &current) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setRunCurrent(current);
    Motor::i().update();
    m_runCurrent = current;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetHoldCurrent(const char *cmd) {
    uint8_t current;
    if ( sscanf_P(cmd, PSTR("set hc %u"), &current) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setHoldCurrent(current);
    Motor::i().update();
    m_holdCurrent = current;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetMotorSteps(const char *cmd) {
    uint16_t steps;
    if ( sscanf_P(cmd, PSTR("set motor steps %u"), &steps) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setMotorSteps(steps);
    Motor::i().update();
    m_motorSteps = steps;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetMicroStepping(const char *cmd) {
    uint8_t steps;
    if ( sscanf_P(cmd, PSTR("set micro steps %u"), &steps) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setMicrostepping(steps);
    Motor::i().update();
    m_microStepping = steps;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetInverted(const char *cmd) {
    bool inverted;
    bool found = false;
    if ( strcmp_P(cmd, F("set invert on")) == 0 ) {
        inverted = true;
        found = true;
    }
    Motor::i().update();
    if ( strcmp_P(cmd, F("set invert off")) == 0 ) {
        inverted = false;
        found = true;
    }
    Motor::i().update();
    if ( ! found ) {
        return false;
    }
    m_invert = inverted;
    Motor::i().setInverted(inverted);
    Motor::i().update();
    saveAndAck();
    Motor::i().update();
    return false;
}

bool Settings::runSetSpeed(const char *cmd) {
    uint16_t speed;
    if ( sscanf_P(cmd, PSTR("set speed %u"), &speed) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setSpeed(speed);
    Motor::i().update();
    m_speed = speed;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetAccel(const char *cmd) {
    uint16_t accel;
    if ( sscanf_P(cmd, PSTR("set accel %u"), &accel) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().setAccel(accel);
    Motor::i().update();
    m_accel = accel;
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetStandStillMode(const char *cmd) {
    TMC2209::StandstillMode mode;
    if ( sscanf_P(cmd, PSTR("set standstill %d"), &mode) != 1 ) {
        return false;
    }
    m_standStillMode = mode;
    Motor::i().setStandStillMode(mode);
    Motor::i().update();
    saveAndAck();
    Motor::i().update();
    return false;
}

bool Settings::runSetStealthChop(const char *cmd) {
    bool enabled;
    bool found = false;
    if ( strcmp_P(cmd, F("set stealthchop on")) == 0 ) {
        enabled = true;
        found = true;
    }
    Motor::i().update();
    if ( strcmp_P(cmd, F("set stealthchop off")) == 0 ) {
        enabled = false;
        found = true;
    }
    Motor::i().update();
    if ( ! found ) {
        return false;
    }
    m_stealthChop = enabled;
    Motor::i().setStealthChop(m_stealthChop);
    Motor::i().update();
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSetCoolStep(const char *cmd) {
    bool enabled;
    bool found = false;
    if ( strcmp_P(cmd, F("set coolstep on")) == 0 ) {
        enabled = true;
        found = true;
    }
    Motor::i().update();
    if ( strcmp_P(cmd, F("set coolstep off")) == 0 ) {
        enabled = false;
        found = true;
    }
    Motor::i().update();
    if ( ! found ) {
        return false;
    }
    m_coolStep = enabled;
    Motor::i().setCoolStep(m_coolStep);
    Motor::i().update();
    saveAndAck();
    Motor::i().update();
    return true;
}

bool Settings::runSync(const char *cmd) {
    uint16_t angle;
    if ( sscanf_P(cmd, PSTR("sync %u"), &angle) != 1 ) {
        return false;
    }
    Motor::i().update();
    Motor::i().syncAngle( (float) angle / 100. );
    Motor::i().update();
    sendStatus();
    Motor::i().update();
    return true;
}

bool Settings::runStop(const char *cmd) {
    if ( strcmp_P(cmd, F("stop") ) ) {
        return false;
    }
    Motor::i().update();
    Motor::i().stop();
    Motor::i().update();
    sendStatus();
    Motor::i().update();
    return true;
}

void Settings::runUnknownCommand() {
  sendErrorMessage(F("Unknown command"));
  Motor::i().update();
}

void Settings::saveAndAck() {
  saveToEEPROM();
  Motor::i().update();
  sendStatus();
  Motor::i().update();
}

bool Settings::runCommand(const char *cmd) {
  Motor::i().update();
  if ( runStatus(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetTarget(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runStop(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetMicroStepping(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetSpeed(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetAccel(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetRunCurrent(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetHoldCurrent(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetMotorSteps(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetStandStillMode(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetInverted(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetTeethSmall(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetTeethBig(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSync(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetStealthChop(cmd) ) {
      return true;
  }
  Motor::i().update();
  if ( runSetCoolStep(cmd) ) {
      return true;
  }
  Motor::i().update();
  runUnknownCommand();
  return false;
}

void Settings::setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);
  Motor::i().update();
}

void Settings::loop() {
  Motor::i().update();
  static bool inCommand = false;
  while ( Serial.available() ) {
    Motor::i().update();
    char c = Serial.read();
    Motor::i().update();
    switch (c) {
      case MSG_PREFIX:
	inCommand = true;
	CommandBuffer::i().clear();
        Motor::i().update();
	CommandBuffer::i().add(c);
        Motor::i().update();
	break;
      case MSG_POSTFIX:
	if ( inCommand ) {
	  inCommand = false;
	  if ( ! CommandBuffer::i().add(c) ) {
            Motor::i().update();
	    // Overflow
	    CommandBuffer::i().clear();
            Motor::i().update();
	    break;
	  }
	  if ( CommandBuffer::i().verifyChecksum() ) {
            Motor::i().update();
	    runCommand(CommandBuffer::i().getCommand());
            Motor::i().update();
	  } else {
	    sendErrorMessage(F("Checksum error"));
	  }
	  CommandBuffer::i().clear();
          Motor::i().update();
	}
	break;
      default:
	if ( inCommand ) {
	  if ( ! CommandBuffer::i().add(c) ) {
            Motor::i().update();
	    // Overflow
	    CommandBuffer::i().clear();
	    inCommand = false;
	  }
	}
    }
  }
}
