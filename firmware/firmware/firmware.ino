#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <util/crc16.h>
#include <TMC2209.h>
#include <AccelStepper.h>

#include "Settings.h"

#define MSG_PREFIX '#'
#define MSG_POSTFIX '$'

void setup() {
    Settings::i().setup();
}

void loop() {
    Settings::i().loop();
}
