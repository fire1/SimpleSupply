#include <Arduino.h>

#include "SimpleSupply.h"


void setup() {
    Serial.begin(9600);
    pinMode(pinVoltPwm, OUTPUT);
    // Pin 9/10 timer setup
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    Serial.println(F("Booting..."));
    pinMode(pinVoltInp, INPUT);
    pinMode(pinAmpsInp, INPUT);
}

unsigned long time,refreshRate = 0;

void loop() {
    time = millis();
    terminal();
    if(time > refreshRate + 300){


    }
}