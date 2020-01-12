#include <Arduino.h>

#include "SimpleSupply.h"

SimpleSupply supply;


void setup() {
    Serial.begin(9600);
    supply.begin();

}

void loop_() {
    u8g2.firstPage();
    do {
        u8g2.drawStr(0,10,"Hello World!");	// write something to the internal memory
    } while (u8g2.nextPage());

    delay(250);
}

void loop() {
    time = millis();
    supply.loop();
    if (time > lastTime + refreshRate) {
        lastTime = time;
        supply.draw();
    }
}