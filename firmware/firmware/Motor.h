#pragma once

#include <math.h>
#include <AccelStepper.h>
#include <TMC2209.h>
#include <ArduinoJson.h>

#define PIN_DIR 6
#define PIN_STEP 7
#define PIN_TX 8
#define PIN_RX 9

typedef uint32_t step_t;

class Motor {
    public:
        static Motor &i();
        void update();
        float gearRatio();
        uint16_t motorRevolutionSteps();
        step_t fullRevolutionSteps();
        float degressPerStep();
        uint16_t stepsPerDegree();
        float stepsToAngle(step_t steps);
        step_t angleToSteps(float angle);
        step_t currentSteps();
        float currentAngle();
        void setTargetAngle(float angle);
        float targetAngle();
        step_t targetSteps();
        void setRunCurrent(uint8_t runCurrent);
        void setHoldCurrent(uint8_t m_holdCurrent);
        uint8_t runCurrent();
        uint8_t holdCurrent();
        void setMicrostepping(uint8_t stepping);
        uint8_t microstepping();
        void setInverted(bool inverted);
        bool isInverted();
        TMC2209::StandstillMode standStillMode();
        void setStandStillMode(TMC2209::StandstillMode mode);
        uint16_t motorSteps();
        void setMotorSteps(uint16_t steps);
        uint16_t speed();
        void setSpeed(uint16_t speed);
        uint16_t teethSmall();
        uint16_t teethBig();
        void setTeethSmall(uint16_t teeth);
        void setTeethBig(uint16_t teeth);
        void setGear(uint16_t teethSmall, uint16_t teethBig, uint16_t motorSteps, uint8_t microstepping);
        void state(char *buff, size_t buffSize);
        void syncSteps(step_t steps);
        void syncAngle(float angle);
        void stop();

    private:
        Motor();
        ~Motor();
        Motor(const Motor &);
        Motor& operator=(const Motor&);

        uint8_t m_runCurrent = 100;
        uint8_t m_holdCurrent = 100;
        bool m_invert = false;
        uint16_t m_speed = 1000;
        TMC2209::StandstillMode m_standStillMode = TMC2209::StandstillMode::NORMAL;

        uint8_t m_microStepping = 16;
        uint16_t m_motorSteps = 200;
        uint16_t m_teethSmall = 20;
        uint16_t m_teethBig = 140;

        float m_targetAngle = 0.0;

        SoftwareSerial *m_serial = nullptr;
        AccelStepper *m_stepper = nullptr;
        TMC2209 m_driver;

        void setTargetSteps(step_t steps);
};
