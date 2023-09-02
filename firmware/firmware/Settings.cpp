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
    Motor::i().setTeethSmall(m_teethSmall);
    Motor::i().setTeethBig(m_teethBig);
    Motor::i().setRunCurrent(m_runCurrent);
    Motor::i().setHoldCurrent(m_holdCurrent);
    Motor::i().setMotorSteps(m_motorSteps);
    Motor::i().setMicrostepping(m_microStepping);
    Motor::i().setInverted(m_invert);
    Motor::i().setStandStillMode(m_standStillMode);
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
  char crcMsg[3];
  crcMsg[0] = ((char *) &crc)[0];
  crcMsg[1] = ((char *) &crc)[1];
  crcMsg[2] = '\0';
  Serial.print(MSG_PREFIX);
  Serial.print(msg);
  Serial.print('\0');
  Serial.print(crcMsg[0]);
  Serial.print(crcMsg[1]);
  Serial.print(MSG_POSTFIX);
}

void Settings::sendMessage(const __FlashStringHelper *msg) {
  uint16_t crc = crcCalc(msg, strlen_P(msg));
  char crcMsg[3];
  crcMsg[0] = ((char *) &crc)[0];
  crcMsg[1] = ((char *) &crc)[1];
  crcMsg[2] = '\0';
  Serial.print(MSG_PREFIX);
  Serial.print(msg);
  Serial.print('\0');
  Serial.print(crcMsg[0]);
  Serial.print(crcMsg[1]);
  Serial.print(MSG_POSTFIX);
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
  Motor::i().state(buff, BUFFSIZE);
  sendMessage(buff);
}

bool Settings::runStatus(const char *cmd) {
  if ( strcmp_P(cmd, F("status") ) ) {
    return false;
  }
  sendStatus();
  return true;
}

bool Settings::runSetTarget(const char *cmd) {
    step_t target;
    if ( sscanf_P(cmd, PSTR("set target %d"), &target) != 1 ) {
        return false;
    }
    Motor::i().setTargetAngle((float) target / 100.);
    sendStatus();
    return true;
}

bool Settings::runSetTeethSmall(const char *cmd) {
    uint16_t teeth;
    if ( sscanf_P(cmd, PSTR("set teeth small %d"), &teeth) != 1 ) {
        return false;
    }
    Motor::i().setTeethSmall(teeth);
    m_teethSmall = teeth;
    saveAndAck();
    return true;
}

bool Settings::runSetTeethBig(const char *cmd) {
    uint16_t teeth;
    if ( sscanf_P(cmd, PSTR("set teeth big %d"), &teeth) != 1 ) {
        return false;
    }
    Motor::i().setTeethBig(teeth);
    m_teethBig = teeth;
    saveAndAck();
    return true;
}

bool Settings::runSetRunCurrent(const char *cmd) {
    uint8_t current;
    if ( sscanf_P(cmd, PSTR("set rc %d"), &current) != 1 ) {
        return false;
    }
    Motor::i().setRunCurrent(current);
    m_runCurrent = current;
    saveAndAck();
    return true;
}

bool Settings::runSetHoldCurrent(const char *cmd) {
    uint8_t current;
    if ( sscanf_P(cmd, PSTR("set hc %d"), &current) != 1 ) {
        return false;
    }
    Motor::i().setHoldCurrent(current);
    m_holdCurrent = current;
    saveAndAck();
    return true;
}

bool Settings::runSetMotorSteps(const char *cmd) {
    uint16_t steps;
    if ( sscanf_P(cmd, PSTR("set motor steps %d"), &steps) != 1 ) {
        return false;
    }
    Motor::i().setMotorSteps(steps);
    m_motorSteps = steps;
    saveAndAck();
    return true;
}

bool Settings::runSetMicroStepping(const char *cmd) {
    uint8_t steps;
    if ( sscanf_P(cmd, PSTR("set micro steps %d"), &steps) != 1 ) {
        return false;
    }
    Motor::i().setMicrostepping(steps);
    m_microStepping = steps;
    saveAndAck();
    return true;
}

bool Settings::runSetInverted(const char *cmd) {
    bool inverted;
    bool found = false;
    if ( strcmp_P(cmd, F("set invert on")) == 0 ) {
        inverted = true;
        found = true;
    }
    if ( strcmp_P(cmd, F("set invert off")) == 0 ) {
        inverted = false;
        found = true;
    }
    if ( ! found ) {
        return false;
    }
    m_invert = inverted;
    Motor::i().setInverted(inverted);
    saveAndAck();
    return false;
}

bool Settings::runSetStandStillMode(const char *cmd) {
    TMC2209::StandstillMode mode;
    if ( sscanf_P(cmd, PSTR("set standstill %d"), &mode) != 1 ) {
        return false;
    }
    m_standStillMode = mode;
    Motor::i().setStandStillMode(mode);
    saveAndAck();
    return false;
}

void Settings::runUnknownCommand() {
  sendErrorMessage(F("Unknown command"));
}

void Settings::saveAndAck() {
  saveToEEPROM();
  sendStatus();
}

bool Settings::runCommand(const char *cmd) {
  if ( runStatus(cmd) ) {
      return true;
  }
  if ( runSetTarget(cmd) ) {
      return true;
  }
  if ( runSetMicroStepping(cmd) ) {
      return true;
  }
  if ( runSetRunCurrent(cmd) ) {
      return true;
  }
  if ( runSetHoldCurrent(cmd) ) {
      return true;
  }
  if ( runSetMotorSteps(cmd) ) {
      return true;
  }
  if ( runSetStandStillMode(cmd) ) {
      return true;
  }
  if ( runSetInverted(cmd) ) {
      return true;
  }
  if ( runSetTeethSmall(cmd) ) {
      return true;
  }
  if ( runSetTeethBig(cmd) ) {
      return true;
  }
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
    switch (c) {
      case MSG_PREFIX:
	inCommand = true;
	CommandBuffer::i().clear();
	CommandBuffer::i().add(c);
	break;
      case MSG_POSTFIX:
	if ( inCommand ) {
	  inCommand = false;
	  if ( ! CommandBuffer::i().add(c) ) {
	    // Overflow
	    CommandBuffer::i().clear();
	    break;
	  }
	  if ( CommandBuffer::i().verifyChecksum() ) {
	    runCommand(CommandBuffer::i().getCommand());
	  } else {
	    sendErrorMessage(F("Checksum error"));
	  }
	  CommandBuffer::i().clear();
	}
	break;
      default:
	if ( inCommand ) {
	  if ( ! CommandBuffer::i().add(c) ) {
	    // Overflow
	    CommandBuffer::i().clear();
	    inCommand = false;
	  }
	}
    }
  }
}