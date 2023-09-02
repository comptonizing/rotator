#include "Motor.h"

Motor::Motor() {
    m_serial = new SoftwareSerial(PIN_RX, PIN_TX);
    m_driver.setup(*m_serial);
    m_stepper = new AccelStepper(1, PIN_STEP, PIN_DIR);

    m_driver.enableCoolStep();
    m_driver.setRunCurrent(m_runCurrent);
    m_driver.setHoldCurrent(m_holdCurrent);
    m_driver.setMicrostepsPerStep(m_microStepping);
    m_driver.setStandstillMode(m_standStillMode);
    m_driver.enable();

    m_stepper->setCurrentPosition(0);
}

void Motor::update() {
    m_stepper->run();
}

float Motor::gearRatio() {
    (float) m_teethBig / (float) m_teethSmall;
}

uint16_t Motor::motorRevolutionSteps() {
    return m_motorSteps * m_microStepping;
}

step_t Motor::fullRevolutionSteps() {
    return (uint16_t) round(gearRatio() * motorRevolutionSteps());
}

float Motor::degressPerStep() {
    return 360. / (float) fullRevolutionSteps();
}

uint16_t Motor::stepsPerDegree() {
    return (uint16_t) round((float) fullRevolutionSteps() / 360.);
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
    StaticJsonDocument<256> json;

    json[F("A")] = currentAngle();
    json[F("S")] = (step_t) currentSteps();
    json[F("M")] = motorRevolutionSteps();
    json[F("F")] = fullRevolutionSteps();
    json[F("SPD")] = stepsPerDegree();
    json[F("TA")] = targetAngle();
    json[F("TS")] = targetSteps();
    json[F("RC")] = runCurrent();
    json[F("HC")] = holdCurrent();
    json[F("uS")] = microstepping();
    json[F("I")] = isInverted();
    json[F("SSM")] = standStillMode();
    json[F("MS")] = motorSteps();
    json[F("GS")] = teethSmall();
    json[F("GB")] = teethBig();
    json[F("MO")] = m_stepper->isRunning();

    serializeJson(json, buff, buffSize);
}
