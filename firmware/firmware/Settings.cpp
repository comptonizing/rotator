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
    Motor::i().setSpeed(m_speed);
    Motor::i().setStandStillMode(m_standStillMode);
    Motor::i().setStealthChop(m_stealthChop);
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
    uint16_t target;
    if ( sscanf_P(cmd, PSTR("set target %u"), &target) != 1 ) {
        return false;
    }
    Motor::i().setTargetAngle((float) target / 100.);
    sendStatus();
    return true;
}

bool Settings::runSetTeethSmall(const char *cmd) {
    uint16_t teeth;
    if ( sscanf_P(cmd, PSTR("set teeth small %u"), &teeth) != 1 ) {
        return false;
    }
    Motor::i().setTeethSmall(teeth);
    m_teethSmall = teeth;
    saveAndAck();
    return true;
}

bool Settings::runSetTeethBig(const char *cmd) {
    uint16_t teeth;
    if ( sscanf_P(cmd, PSTR("set teeth big %u"), &teeth) != 1 ) {
        return false;
    }
    Motor::i().setTeethBig(teeth);
    m_teethBig = teeth;
    saveAndAck();
    return true;
}

bool Settings::runSetRunCurrent(const char *cmd) {
    uint8_t current;
    if ( sscanf_P(cmd, PSTR("set rc %u"), &current) != 1 ) {
        return false;
    }
    Motor::i().setRunCurrent(current);
    m_runCurrent = current;
    saveAndAck();
    return true;
}

bool Settings::runSetHoldCurrent(const char *cmd) {
    uint8_t current;
    if ( sscanf_P(cmd, PSTR("set hc %u"), &current) != 1 ) {
        return false;
    }
    Motor::i().setHoldCurrent(current);
    m_holdCurrent = current;
    saveAndAck();
    return true;
}

bool Settings::runSetMotorSteps(const char *cmd) {
    uint16_t steps;
    if ( sscanf_P(cmd, PSTR("set motor steps %u"), &steps) != 1 ) {
        return false;
    }
    Motor::i().setMotorSteps(steps);
    m_motorSteps = steps;
    saveAndAck();
    return true;
}

bool Settings::runSetMicroStepping(const char *cmd) {
    uint8_t steps;
    if ( sscanf_P(cmd, PSTR("set micro steps %u"), &steps) != 1 ) {
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

bool Settings::runSetSpeed(const char *cmd) {
    uint16_t speed;
    if ( sscanf_P(cmd, PSTR("set speed %u"), &speed) != 1 ) {
        return false;
    }
    Motor::i().setSpeed(speed);
    m_speed = speed;
    saveAndAck();
    return true;
}

bool Settings::runSetAccel(const char *cmd) {
    uint16_t accel;
    if ( sscanf_P(cmd, PSTR("set accel %u"), &accel) != 1 ) {
        return false;
    }
    Motor::i().setAccel(accel);
    m_accel = accel;
    saveAndAck();
    return true;
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

bool Settings::runSetStealthChop(const char *cmd) {
    bool enabled;
    bool found;
    if ( strcmp_P(cmd, F("set stealthchop on")) == 0 ) {
        enabled = true;
        found = true;
    }
    if ( strcmp_P(cmd, F("set stealthchop off")) == 0 ) {
        enabled = false;
        found = true;
    }
    if ( ! found ) {
        return false;
    }
    m_stealthChop = enabled;
    Motor::i().setStealthChop(m_stealthChop);
    saveAndAck();
    return true;
}

bool Settings::runSync(const char *cmd) {
    uint16_t angle;
    if ( sscanf_P(cmd, PSTR("sync %u"), &angle) != 1 ) {
        return false;
    }
    Motor::i().syncAngle( (float) angle / 100. );
    sendStatus();
    return true;
}

bool Settings::runStop(const char *cmd) {
    if ( strcmp_P(cmd, F("stop") ) ) {
        return false;
    }
    Motor::i().stop();
    sendStatus();
    return true;
}

void Settings::runUnknownCommand() {
  sendErrorMessage(F("Unknown command"));
}

void Settings::saveAndAck() {
  saveToEEPROM();
  sendStatus();
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
        Motor::i().update();
	inCommand = true;
	CommandBuffer::i().clear();
	CommandBuffer::i().add(c);
	break;
      case MSG_POSTFIX:
        Motor::i().update();
	if ( inCommand ) {
	  inCommand = false;
	  if ( ! CommandBuffer::i().add(c) ) {
	    // Overflow
	    CommandBuffer::i().clear();
	    break;
	  }
	  if ( CommandBuffer::i().verifyChecksum() ) {
            Motor::i().update();
	    runCommand(CommandBuffer::i().getCommand());
	  } else {
	    sendErrorMessage(F("Checksum error"));
	  }
	  CommandBuffer::i().clear();
	}
	break;
      default:
        Motor::i().update();
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
