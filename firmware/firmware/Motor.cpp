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



#include "Motor.h"

Motor::Motor() {
    m_serial = new SoftwareSerial(PIN_RX, PIN_TX);
    m_driver.setup(*m_serial);
    m_stepper = new AccelStepper(1, PIN_STEP, PIN_DIR);

	enable();
    m_driver.setRunCurrent(m_runCurrent);
    m_driver.setHoldCurrent(m_holdCurrent);
    m_driver.setStandstillMode(m_standStillMode);
    m_driver.disableAnalogCurrentScaling();
    if ( m_coolStep ) {
        m_driver.enableCoolStep();
    } else {
        m_driver.disableCoolStep();
    }
    if ( m_stealthChop ) {
        m_driver.enableStealthChop();
    } else {
        m_driver.disableStealthChop();
    }

	setGear(m_teethSmall, m_teethBig, m_motorSteps, m_microStepping);

    m_stepper->setCurrentPosition(0);
    m_stepper->setSpeed(m_speed);
    m_stepper->setAcceleration(m_accel);
}

void Motor::update() {
    m_stepper->run();
	if ( ! m_stepper->isRunning() ) {
		if ( m_holdCurrent == 0 ) {
			disable();
		} else {
			enable();
		}
	}
}

float Motor::gearRatio() {
    return (float) m_teethBig / (float) m_teethSmall;
}

uint16_t Motor::motorRevolutionSteps() {
    return m_motorSteps * m_microStepping;
}

step_t Motor::fullRevolutionSteps() {
    return (step_t) round(gearRatio() * (float) motorRevolutionSteps());
}

float Motor::degressPerStep() {
    return 360. / (float) fullRevolutionSteps();
}

float Motor::stepsPerDegree() {
    return (float) fullRevolutionSteps() / 360.;
}

float Motor::stepsToAngle(step_t steps) {
    return (float) steps / (float) fullRevolutionSteps() * 360.;
}

step_t Motor::angleToSteps(float angle) {
    return (step_t) round(angle / 360. * fullRevolutionSteps());
}

step_t Motor::currentSteps() {
    return m_stepper->currentPosition();
}

float Motor::currentAngle() {
    return stepsToAngle(currentSteps());
}

void Motor::setTargetSteps(step_t steps) {
	enable();
    m_stepper->moveTo(steps);
}

void Motor::setTargetAngle(float angle) {
    m_targetAngle = angle;
    setTargetSteps(angleToSteps(m_targetAngle));
}

float Motor::targetAngle() {
    return m_targetAngle;
}

step_t Motor::targetSteps() {
    return m_stepper->targetPosition();
}

void Motor::setRunCurrent(uint8_t runCurrent) {
    m_runCurrent = runCurrent;
    m_driver.setRunCurrent(m_runCurrent);
}

void Motor::setHoldCurrent(uint8_t holdCurrent) {
    m_holdCurrent = holdCurrent;
    m_driver.setHoldCurrent(m_holdCurrent);
}

uint8_t Motor::runCurrent() {
    return m_runCurrent;
}

uint8_t Motor::holdCurrent() {
    return m_holdCurrent;
}

void Motor::setMicrostepping(uint8_t stepping) {
    setGear(m_teethSmall, m_teethBig, m_motorSteps, stepping);
}

uint8_t Motor::microstepping() {
    return m_microStepping;
}

void Motor::setInverted(bool inverted) {
    if ( inverted ) {
        if ( ! m_invert ) {
            m_driver.enableInverseMotorDirection();
            m_stepper->setCurrentPosition(fullRevolutionSteps() - m_stepper->currentPosition());
            m_invert = true;
        }
    } else {
        if ( m_invert ) {
            m_driver.disableInverseMotorDirection();
            m_stepper->setCurrentPosition(fullRevolutionSteps() - m_stepper->currentPosition());
            m_invert = false;
        }
    }
}

bool Motor::isInverted() {
    return m_invert;
}

TMC2209::StandstillMode Motor::standStillMode() {
    return m_standStillMode;
}

void Motor::setStandStillMode(TMC2209::StandstillMode mode) {
    m_standStillMode = mode;
    m_driver.setStandstillMode(m_standStillMode);
}

uint16_t Motor::motorSteps() {
    return m_motorSteps;
}

void Motor::setMotorSteps(uint16_t steps) {
    setGear(m_teethSmall, m_teethBig, steps, m_microStepping);
}

uint16_t Motor::speed() {
    return m_speed;
}

void Motor::setSpeed(uint16_t speed) {
    m_speed = speed;
    m_stepper->setMaxSpeed(m_speed);
}

uint16_t Motor::accel() {
    return m_accel;
}

void Motor::setAccel(uint16_t accel) {
    m_accel = accel;
    m_stepper->setAcceleration(m_accel);
}

uint16_t Motor::teethSmall() {
    return m_teethSmall;
}

uint16_t Motor::teethBig() {
    return m_teethBig;
}

void Motor::setTeethSmall(uint16_t teeth) {
    setGear(teeth, m_teethBig, m_motorSteps, m_microStepping);
}

void Motor::setTeethBig(uint16_t teeth) {
    setGear(m_teethSmall, teeth, m_motorSteps, m_microStepping);
}

void Motor::setGear(uint16_t teethSmall, uint16_t teethBig, uint16_t motorSteps, uint8_t microstepping) {
    step_t oldSteps = currentSteps();
    step_t oldStepsPerRevolution = fullRevolutionSteps();
    step_t newStepsPerRevolution = (step_t) round((float) teethBig / (float) teethSmall * motorSteps * microstepping);
    step_t newSteps = (step_t) (float) oldSteps / (float) oldStepsPerRevolution * (float) newStepsPerRevolution;

    m_teethSmall = teethSmall;
    m_teethBig = teethBig;
    m_motorSteps = motorSteps;
    m_microStepping = microstepping;
    m_stepper->setCurrentPosition(newSteps);
    m_driver.setMicrostepsPerStep(m_microStepping);
}

Motor &Motor::i() {
    static Motor theInstance;
    return theInstance;
}

void Motor::state(char *buff, size_t buffSize) {
    // Make sure all parameters are current
    update();
    StaticJsonDocument<256> json;
    update();

    json[F("A")] = currentAngle();
    update();
    json[F("S")] = (step_t) currentSteps();
    update();
    json[F("M")] = motorSteps();
    update();
    json[F("F")] = fullRevolutionSteps();
    update();
    json[F("SPD")] = stepsPerDegree();
    update();
    json[F("TA")] = targetAngle();
    update();
    json[F("TS")] = targetSteps();
    update();
    json[F("RC")] = runCurrent();
    update();
    json[F("HC")] = holdCurrent();
    update();
    json[F("uS")] = microstepping();
    update();
    json[F("I")] = isInverted();
    update();
    json[F("SSM")] = standStillMode();
    update();
    json[F("MS")] = motorSteps();
    update();
    json[F("SP")] = speed();
    update();
    json[F("AC")] = accel();
    update();
    json[F("GS")] = teethSmall();
    update();
    json[F("GB")] = teethBig();
    update();
    json[F("MO")] = m_stepper->isRunning();
    update();
    json[F("CP")] = stealthChop();
    update();
    json[F("CS")] = coolStep();

    update();

    serializeJson(json, buff, buffSize);

    update();
}

void Motor::syncSteps(step_t steps) {
    m_stepper->setCurrentPosition(steps);
}

void Motor::syncAngle(float angle) {
    m_stepper->setCurrentPosition(angleToSteps(angle));
}

void Motor::stop() {
    m_stepper->stop();
}

bool Motor::stealthChop() {
    return m_stealthChop;
}

void Motor::setStealthChop(bool enabled) {
    m_stealthChop = enabled;
    if ( m_stealthChop ) {
        m_driver.enableStealthChop();
    } else {
        m_driver.disableStealthChop();
    }
}

bool Motor::coolStep() {
    return m_coolStep;
}

void Motor::setCoolStep(bool enabled) {
    m_coolStep = enabled;
    if ( m_coolStep ) {
        m_driver.enableCoolStep();
    } else {
        m_driver.disableCoolStep();
    }
}

void Motor::enable() {
	if ( m_enabled ) {
		return;
	}
	m_driver.enable();
	m_enabled = true;
}

void Motor::disable() {
	if ( ! m_enabled ) {
		return;
	}
	m_driver.disable();
	m_enabled = false;
}

bool Motor::isEnabled() {
	return m_enabled;
}
