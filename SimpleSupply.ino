#include <Arduino.h>

#include "SimpleSupply.h"

SimpleSupply supply;

#define noDisplay

void setup() {
    Serial.begin(9600);
    supply.begin();

}


void loop() {
    time = millis();
    supply.loop();
    if (time > lastTime + refreshRate) {
        lastTime = time;
        supply.draw();
    }
}