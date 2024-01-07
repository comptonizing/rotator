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


#include "rotator.h"
#include <indirotator.h>

static std::unique_ptr<RotatorPollux> rotatorPolluxDriver((new RotatorPollux()));

RotatorPollux::RotatorPollux() {
    setVersion(0, 3);
}

const char *RotatorPollux::getDefaultName() {
    return "Pollux Rotator";
}

bool RotatorPollux::loadConfig(bool silent, const char *property) {
    // Everything ignored here, config comes from the devices itself
    (void) silent;
    (void) property;
    return true;
}

bool RotatorPollux::saveConfigItems(FILE *fp) {
    (void) fp;
    return true;
}

bool RotatorPollux::initProperties() {
    INDI::Rotator::initProperties();
    SetCapability(
            ROTATOR_CAN_REVERSE |
            ROTATOR_CAN_SYNC |
            ROTATOR_CAN_ABORT
            );
    addAuxControls();
    addDebugControl();
    addConfigurationControl();
    setDefaultPollingPeriod(500);
    addPollPeriodControl();

    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
    serialConnection->registerHandshake([&]() {
        return Handshake();
        });
    registerConnection(serialConnection);

    IUFillNumber(&MotorN[SPEED], "SPEED", "Speed", "%.0f", 1, 200, 1, 20);
    IUFillNumber(&MotorN[ACCEL], "ACCEL", "Acceleration", "%.0f", 1, 200, 1, 20);
    IUFillNumber(&MotorN[RUNCURRENT], "RUNCURRENT", "Run Current [%]", "%.0f", 1, 100, 1, 100);
    IUFillNumber(&MotorN[HOLDCURRENT], "HOLDCURRENT", "Hold Current [%]", "%.0f", 0, 100, 1, 10);
    IUFillNumber(&MotorN[STEPS_PER_MOTOR_REVOLUTION], "STEPS_PER_MOTOR_REVOLUTION",
            "Steps per revolution", "%.0f", 1, 10000, 1, 200);
    IUFillNumberVector(&MotorNP, MotorN, 5, getDeviceName(), "MOTOR", "Motor Settings",
            MAIN_CONTROL_TAB, IP_RW, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&MicroSteppingS[MS1], "MS1", "1", ISS_ON);
    IUFillSwitch(&MicroSteppingS[MS2], "MS2", "2", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS4], "MS4", "4", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS8], "MS8", "8", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS16], "MS16", "16", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS32], "MS32", "32", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS64], "MS64", "64", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS128], "MS128", "128", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS256], "MS256", "256", ISS_OFF);
    IUFillSwitchVector(&MicroSteppingSP, MicroSteppingS, 9, getDeviceName(),
            "MICROSTEPPING", "Microstepping", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&StandStillModeS[NORMAL], "NORMAL", "Normal", ISS_ON);
    IUFillSwitch(&StandStillModeS[FREEWHEELING], "FREEWHEELING", "Freewheeling", ISS_OFF);
    IUFillSwitch(&StandStillModeS[BREAKING], "BREAKING", "Breaking", ISS_OFF);
    IUFillSwitch(&StandStillModeS[STRONG_BREAKING], "STRONG_BREAKING", "Strong Breaking", ISS_OFF);
    IUFillSwitchVector(&StandStillModeSP, StandStillModeS, 4, getDeviceName(),
            "STANDSTILL", "Standstill Mode", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&StealthChopS[INDI_ENABLED], "ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&StealthChopS[INDI_DISABLED], "DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&StealthChopSP, StealthChopS, 2, getDeviceName(),
            "STEALTHCHOP", "Stealth Chop", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&CoolStepS[INDI_ENABLED], "ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&CoolStepS[INDI_DISABLED], "DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&CoolStepSP, CoolStepS, 2, getDeviceName(),
            "COOLSTEP", "Cool Step", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillNumber(&GearN[TEETH_SMALL], "TEETH_SMALL", "Teeth small", "%.0f", 1, 100, 1, 10);
    IUFillNumber(&GearN[TEETH_BIG], "TEETH_BIG", "Teeth big", "%.0f", 1, 1000, 1, 170);
    IUFillNumberVector(&GearNP, GearN, 2, getDeviceName(), "GEAR", "Gear Settings",
            MAIN_CONTROL_TAB, IP_RW, TIMEOUT, IPS_IDLE);

    IUFillNumber(&MotionN[STEPS], "STEPS", "Steps", "%.0f", 0, 4294967295, 1, 0);
    IUFillNumber(&MotionN[STEPS_PER_REVOLUTION], "STEPS_PER_REVOLUTION",
            "Steps per revolution", "%.0f", 1, 4294967295, 1, 10000);
    IUFillNumber(&MotionN[STEPS_PER_DEGREE], "STEPS_PER_DEGREE",
            "Steps per degree", "%.2f", 1, 4294967295, 1, 100);
    IUFillNumberVector(&MotionNP, MotionN, 3, getDeviceName(), "MOTION",
            "Motion Status", MAIN_CONTROL_TAB, IP_RO, TIMEOUT, IPS_IDLE);

    GotoRotatorN[0].min = 0.0;
    GotoRotatorN[0].max = 359.99;
    GotoRotatorN[0].step = 0.01;

    return true;
}

bool RotatorPollux::updateProperties() {
    INDI::Rotator::updateProperties();
    if ( isConnected() ) {
        defineProperty(&MotorNP);
        defineProperty(&MicroSteppingSP);
        defineProperty(&StandStillModeSP);
        defineProperty(&StealthChopSP);
        defineProperty(&CoolStepSP);
        defineProperty(&GearNP);
        defineProperty(&MotionNP);
    } else {
        deleteProperty(MotorNP.name);
        deleteProperty(MicroSteppingSP.name);
        deleteProperty(StandStillModeSP.name);
        deleteProperty(StealthChopSP.name);
        deleteProperty(CoolStepSP.name);
        deleteProperty(GearNP.name);
        deleteProperty(MotionNP.name);
    }
    return true;
}

bool RotatorPollux::Handshake() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  json data;
  try {
    data = json::parse(rsp);
  } catch (...) {
    LOG_ERROR("JSON parse error");
    return false;
  }
  if ( data.contains("Error") ) {
    LOGF_ERROR("Device error: %s", data["Error"].template get<std::string>().c_str());
    return false;
  }
  return true;
}

IPState RotatorPollux::MoveRotator(double angle) {
    char rsp[RSPBUFF];
    char cmd[CMDBUFF];
    snprintf(cmd, CMDBUFF, "set target %d", static_cast<int>(round(100. * angle)));
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        return IPS_ALERT;
    }
    return GotoRotatorNP.s;
}

void RotatorPollux::TimerHit() {
  if ( isConnected() ) {
    update();
  }
  SetTimer(getCurrentPollingPeriod());
}

bool RotatorPollux::ReverseRotator(bool enabled) {
    bool commandStatus;
    char rsp[RSPBUFF];
    if ( enabled ) {
        commandStatus = sendCommand("set invert on", rsp);
    } else {
        commandStatus = sendCommand("set invert off", rsp);
    }
    if ( ! commandStatus || ! updateFromResponse(rsp) ) {
        return false;
    }
    return true;
}

bool RotatorPollux::SyncRotator(double angle) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    snprintf(cmd, CMDBUFF, "sync %d", static_cast<int>(round(angle * 100.)));
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        return false;
    }
    return true;
}

bool RotatorPollux::AbortRotator() {
    LOG_INFO("Aborting rotation");
    char cmd[] = "stop";
    char rsp[RSPBUFF];
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        return false;
    }
    return true;
}

uint16_t RotatorPollux::crc16_update(uint16_t crc, uint8_t a) {
  // Code from crc16.h from the Arduino core tools
  crc ^= a;
  for (int ii=0; ii<8; ii++) {
    if ( crc & 1 ) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

uint16_t RotatorPollux::crcCalc(const void *data, size_t n) {
  const uint8_t *ptr = static_cast<const uint8_t *>(data);
  uint16_t crc = 0;
  for (size_t ii=0; ii<n; ii++) {
    crc = crc16_update(crc, ptr[ii]);
  }
  return crc;
}

uint16_t RotatorPollux::crcCalc(const char *str) {
  return crcCalc(static_cast<const void *>(str), strlen(str));
}

void RotatorPollux::cmdCrc(const char *cmd, char *out) {
  uint16_t crc = crcCalc(cmd);
  int len = strlen(cmd);
  memcpy(out+1, cmd, len);
  out[0] = '#';
  out[len+1] = '\0';
  out[len+2] = ((char *) &crc)[0];
  if ( out[len+2] == '$' ) {
    out[len+2] = '1';
  }
  out[len+3] = ((char *) &crc)[1];
  if ( out[len+3] == '$' ) {
    out[len+3] = '1';
  }
  out[len+4] = '$';
  out[len+5] = '\0';
}

bool RotatorPollux::checkCrc(const char *rsp) {
  uint16_t crcGotten;
  size_t len = strlen(rsp);
  ((char *) &crcGotten)[0] = rsp[len+1];
  ((char *) &crcGotten)[1] = rsp[len+2];
  uint16_t crcCalculated = crcCalc((const void *) rsp, len);
  char *crcChar = (char *) &crcCalculated;
  if ( crcChar[0] == '$' ) {
    crcChar[0] = '1';
  }
  if ( crcChar[1] == '$' ) {
    crcChar[1] = '1';
  }
  if ( crcGotten == crcCalculated ) {
    return true;
  } else {
    LOG_ERROR("Checksum error");
    LOGF_ERROR("Message: %s", rsp);
    LOGF_ERROR("Checksum: %d (0x%02x 0x%02x), expected %d (0x%02x 0x%02x)",
    crcCalculated, ((unsigned char *) &crcCalculated)[0], ((unsigned char *) &crcCalculated)[1],
    crcGotten, ((unsigned char *) &crcGotten)[0], ((unsigned char *) &crcGotten)[1]);
    return false;
  }
}

bool RotatorPollux::sendCommand(const char *cmd, char *rsp) {
  LOGF_DEBUG("Sending command: %s", cmd);
  int nbytes_written = 0, nbytes_read = 0, rc = -1;
  int PortFD = serialConnection->getPortFD();
  LOGF_DEBUG("PortFD: %d", PortFD);
  char buff[CMDBUFF];
  char err[ERRBUFF];
  char rspBuff[RSPBUFF];
  cmdCrc(cmd, buff);

  tcflush(PortFD, TCIOFLUSH);
  if ( (rc = tty_write(PortFD, buff, strlen(buff)+4, &nbytes_written)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error writing command %s: %s", cmd, err);
    return false;
  }

  LOG_DEBUG("RCV");

  // Somtimes there's garbage on the line so read until the next #
  rspBuff[0] = '\0';
  while ( rspBuff[0] != '#' ) {
    if ( (rc = tty_read(PortFD, rspBuff, 1, TIMEOUT, &nbytes_read) ) != TTY_OK ) {
      tty_error_msg(rc, err, ERRBUFF);
      LOGF_ERROR("Error reading response: %s", err);
      return false;
    }
  }

  if ( (rc = tty_read_section(PortFD, rspBuff+1, '$', TIMEOUT, &nbytes_read)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error reading response: %s", err);
    return false;
  }

  memcpy(rsp, rspBuff+1, strlen(rspBuff)+2);
  if ( ! checkCrc(rsp) ) {
    return false;
  }
  return true;
}

void RotatorPollux::setAngle(const json& data) {
    try {
        double currentAngle = data["A"].template get<double>();
        int currentSteps = data["S"].template get<int>();
        int targetSteps = data["TS"].template get<int>();
        bool moving = data["MO"].template get<bool>();

        GotoRotatorN[0].value = currentAngle;
        if ( moving ) {
            GotoRotatorNP.s = IPS_BUSY;
        } else {
            if ( currentSteps == targetSteps ) {
                GotoRotatorNP.s = IPS_OK;
            } else {
                GotoRotatorNP.s = IPS_ALERT;
            }
        }
        IDSetNumber(&GotoRotatorNP, nullptr);
    } catch (...) {
        GotoRotatorNP.s = IPS_ALERT;
        IDSetNumber(&GotoRotatorNP, nullptr);
        throw;
    }
}

void RotatorPollux::setReverse(const json& data) {
    try {
        bool inverted = data["I"].template get<bool>();
        ReverseRotatorS[INDI_ENABLED].s = inverted ? ISS_ON : ISS_OFF;
        ReverseRotatorS[INDI_DISABLED].s = inverted ? ISS_OFF : ISS_ON;
        ReverseRotatorSP.s = IPS_OK;
        IDSetSwitch(&ReverseRotatorSP, nullptr);
    } catch (...) {
        ReverseRotatorSP.s = IPS_ALERT;
        IDSetSwitch(&ReverseRotatorSP, nullptr);
        throw;
    }
}

void RotatorPollux::setStealthChop(const json& data) {
    try {
        bool enabled = data["CP"].template get<bool>();
        StealthChopS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
        StealthChopS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
        StealthChopSP.s = IPS_OK;
        IDSetSwitch(&StealthChopSP, nullptr);
    } catch (...) {
        StealthChopSP.s = IPS_ALERT;
        IDSetSwitch(&StealthChopSP, nullptr);
        throw;
    }
}

void RotatorPollux::setCoolStep(const json& data) {
    try {
        bool enabled = data["CS"].template get<bool>();
        CoolStepS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
        CoolStepS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
        CoolStepSP.s = IPS_OK;
        IDSetSwitch(&CoolStepSP, nullptr);
    } catch (...) {
        CoolStepSP.s = IPS_ALERT;
        IDSetSwitch(&CoolStepSP, nullptr);
        throw;
    }
}

void RotatorPollux::setMotor(const json& data) {
    try {
        MotorN[SPEED].value = data["SP"].template get<int>();
        MotorN[ACCEL].value = data["AC"].template get<int>();
        MotorN[RUNCURRENT].value = data["RC"].template get<int>();
        MotorN[HOLDCURRENT].value = data["HC"].template get<int>();
        MotorN[STEPS_PER_MOTOR_REVOLUTION].value = data["M"].template get<int>();
    } catch (...) {
        MotorNP.s = IPS_ALERT;
        IDSetNumber(&MotorNP, nullptr);
        throw;
    }
    MotorNP.s = IPS_OK;
    IDSetNumber(&MotorNP, nullptr);
}

void RotatorPollux::setMicrostepping(const json& data) {
    try {
        uint8_t ms = data["uS"].template get<int>();
        for (uint8_t ii=0; ii<9; ii++) {
            if ( ms & ( 0x1 << ii) ) {
                MicroSteppingS[ii].s = ISS_ON;
            } else {
                MicroSteppingS[ii].s = ISS_OFF;
            }
        }
    } catch (...) {
      MicroSteppingSP.s = IPS_ALERT;
      IDSetSwitch(&MicroSteppingSP, nullptr);
      throw;
    }
    MicroSteppingSP.s = IPS_OK;
    IDSetSwitch(&MicroSteppingSP, nullptr);
}

void RotatorPollux::setStandstillMode(const json& data) {
    try {
        uint8_t mode = data["SSM"].template get<int>();
        for (uint8_t ii=0; ii<4; ii++) {
            StandStillModeS[ii].s = mode == ii ? ISS_ON : ISS_OFF;
        }
    } catch (...) {
        StandStillModeSP.s = IPS_ALERT;
        IDSetSwitch(&StandStillModeSP, nullptr);
        throw;
    }
    StandStillModeSP.s = IPS_OK;
    IDSetSwitch(&StandStillModeSP, nullptr);
}

void RotatorPollux::setGear(const json& data) {
    try {
        GearN[TEETH_SMALL].value = data["GS"].template get<int>();
        GearN[TEETH_BIG].value = data["GB"].template get<int>();
    } catch (...) {
        GearNP.s = IPS_ALERT;
        IDSetNumber(&GearNP, nullptr);
        throw;
    }
    GearNP.s = IPS_OK;
    IDSetNumber(&GearNP, nullptr);
}

void RotatorPollux::setMotion(const json& data) {
    try {
        MotionN[STEPS].value = data["S"].template get<int>();
        MotionN[STEPS_PER_REVOLUTION].value = data["F"].template get<int>();
        MotionN[STEPS_PER_DEGREE].value = data["SPD"].template get<double>();
    } catch (...) {
        MotionNP.s = IPS_ALERT;
        IDSetNumber(&MotionNP, nullptr);
        throw;
    }
    MotionNP.s = IPS_OK;
    IDSetNumber(&MotionNP, nullptr);
}

bool RotatorPollux::updateFromResponse(const char *rsp) {
  json data;
  try {
    data = json::parse(rsp);
  } catch (...) {
    LOGF_ERROR("Error parsing JSON: %s", rsp);
    return false;
  }
  if ( data.contains("Error") ) {
    LOGF_ERROR("Device error: %s", data["Error"].template get<std::string>().c_str());
    return false;
  }
  try {
      setAngle(data);
      setReverse(data);
      setStealthChop(data);
      setCoolStep(data);
      setMotor(data);
      setMicrostepping(data);
      setStandstillMode(data);
      setGear(data);
      setMotion(data);
  } catch (...) {
    LOG_ERROR("Could not decode values from device");
    return false;
  }
  return true;
}

bool RotatorPollux::update() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  return updateFromResponse(rsp);
}

bool RotatorPollux::processMotorNP(double *values, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    for (int ii=0; ii<n; ii++) {
        for (int jj=0; jj<n; jj++) {
            if ( strcmp(MotorN[ii].name, names[jj]) == 0 ) {
                if ( MotorN[ii].value != values[jj] ) {
                    if ( strcmp(names[jj], "SPEED") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set speed %d", static_cast<int>(values[jj]));
                    }
                    if ( strcmp(names[jj], "ACCEL") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set accel %d", static_cast<int>(values[jj]));
                    }
                    if ( strcmp(names[jj], "RUNCURRENT") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set rc %d", static_cast<int>(values[jj]));
                    }
                    if ( strcmp(names[jj], "HOLDCURRENT") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set hc %d", static_cast<int>(values[jj]));
                    }
                    if ( ! sendCommand(cmd, rsp) ) {
                        MotorNP.s = IPS_ALERT;
                        IDSetNumber(&MotorNP, nullptr);
                        return false;
                    }
                }
            }
        }
    }
    if ( ! update() ) {
        MotorNP.s = IPS_ALERT;
        IDSetNumber(&MotorNP, nullptr);
        return false;
    }
    MotorNP.s = IPS_OK;
    IDSetNumber(&MotorNP, nullptr);
    return true;
}

bool RotatorPollux::processGearNP(double *values, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    for (int ii=0; ii<n; ii++) {
        for (int jj=0; jj<n; jj++) {
            if ( strcmp(GearN[ii].name, names[jj]) == 0 ) {
                if ( GearN[ii].value != values[jj] ) {
                    if ( strcmp(names[jj], "TEETH_SMALL") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set teeth small %d", static_cast<int>(values[jj]));
                    }
                    if ( strcmp(names[jj], "TEETH_BIG") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set teeth big %d", static_cast<int>(values[jj]));
                    }
                    if ( ! sendCommand(cmd, rsp) ) {
                        GearNP.s = IPS_ALERT;
                        IDSetNumber(&GearNP, nullptr);
                        return false;
                    }
                }
            }
        }
    }
    if ( ! update() ) {
        GearNP.s = IPS_ALERT;
        IDSetNumber(&GearNP, nullptr);
        return false;
    }
    GearNP.s = IPS_OK;
    IDSetNumber(&GearNP, nullptr);
    return true;
}

bool RotatorPollux::processMicroSteppingSP(ISState *states, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    int old = IUFindOnSwitchIndex(&MicroSteppingSP);
    IUUpdateSwitch(&MicroSteppingSP, states, names, n);
    int active = IUFindOnSwitchIndex(&MicroSteppingSP);
    snprintf(cmd, CMDBUFF, "set micro steps %d", (0x1 <<  active));
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        MicroSteppingS[active].s = ISS_OFF;
        MicroSteppingS[old].s = ISS_ON;
        MicroSteppingSP.s = IPS_ALERT;
        IDSetSwitch(&MicroSteppingSP, nullptr);
        return false;
    }
    MicroSteppingSP.s = IPS_OK;
    IDSetSwitch(&MicroSteppingSP, nullptr);
    return true;
}

bool RotatorPollux::processStandStillModeSP(ISState *states, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    int old = IUFindOnSwitchIndex(&StandStillModeSP);
    IUUpdateSwitch(&StandStillModeSP, states, names, n);
    int active = IUFindOnSwitchIndex(&StandStillModeSP);
    snprintf(cmd, CMDBUFF, "set standstill %d", active);
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        StandStillModeS[active].s = ISS_OFF;
        StandStillModeS[old].s = ISS_ON;
        StandStillModeSP.s = IPS_ALERT;
        IDSetSwitch(&StandStillModeSP, nullptr);
        return false;
    }
    StandStillModeSP.s = IPS_OK;
    IDSetSwitch(&StandStillModeSP, nullptr);
    return true;
}

bool RotatorPollux::processStealthChopSP(ISState *states, char *names[], int n) {
    char rsp[RSPBUFF];
    bool commandStatus;
    int old = IUFindOnSwitchIndex(&StealthChopSP);
    IUUpdateSwitch(&StealthChopSP, states, names, n);
    int active = IUFindOnSwitchIndex(&StealthChopSP);
    if ( active == INDI_ENABLED ) {
        commandStatus = sendCommand("set stealthchop on", rsp);
    } else {
        commandStatus = sendCommand("set stealthchop off", rsp);
    }
    if ( ! commandStatus || ! updateFromResponse(rsp) ) {
        StealthChopS[active].s = ISS_OFF;
        StealthChopS[old].s = ISS_ON;
        StealthChopSP.s = IPS_ALERT;
        IDSetSwitch(&StealthChopSP, nullptr);
        return false;
    }
    StealthChopSP.s = IPS_OK;
    IDSetSwitch(&StealthChopSP, nullptr);
    return true;
}

bool RotatorPollux::processCoolStepSP(ISState *states, char *names[], int n) {
    char rsp[RSPBUFF];
    bool commandStatus;
    int old = IUFindOnSwitchIndex(&CoolStepSP);
    IUUpdateSwitch(&CoolStepSP, states, names, n);
    int active = IUFindOnSwitchIndex(&CoolStepSP);
    if ( active == INDI_ENABLED ) {
        commandStatus = sendCommand("set coolstep on", rsp);
    } else {
        commandStatus = sendCommand("set coolstep off", rsp);
    }
    if ( ! commandStatus || ! updateFromResponse(rsp) ) {
        CoolStepS[active].s = ISS_OFF;
        CoolStepS[old].s = ISS_ON;
        CoolStepSP.s = IPS_ALERT;
        IDSetSwitch(&CoolStepSP, nullptr);
        return false;
    }
    CoolStepSP.s = IPS_OK;
    IDSetSwitch(&CoolStepSP, nullptr);
    return true;
}

bool RotatorPollux::ISNewNumber(const char *dev, const char *name, double *values, char *names[], int n) {
    if (dev && !strcmp(dev, getDeviceName())) {
        if ( strcmp(name, MotorNP.name) == 0 ) {
            return processMotorNP(values, names, n);
        }
        if ( strcmp(name, GearNP.name) == 0 ) {
            return processGearNP(values, names, n);
        }
        return INDI::Rotator::ISNewNumber(dev, name, values, names, n);
    }
    return INDI::Rotator::ISNewNumber(dev, name, values, names, n);
}

bool RotatorPollux::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) {
    if (dev && !strcmp(dev, getDeviceName())) {
        if ( strcmp(name, MicroSteppingSP.name) == 0 ) {
            return processMicroSteppingSP(states, names, n);
        }
        if ( strcmp(name, StandStillModeSP.name) == 0 ) {
            return processStandStillModeSP(states, names, n);
        }
        if ( strcmp(name, StealthChopSP.name) == 0 ) {
            return processStealthChopSP(states, names, n);
        }
        if ( strcmp(name, CoolStepSP.name) == 0 ) {
            return processCoolStepSP(states, names, n);
        }
        return INDI::Rotator::ISNewSwitch(dev, name, states, names, n);
    }
    return INDI::Rotator::ISNewSwitch(dev, name, states, names, n);
}
