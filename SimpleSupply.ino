#include <Arduino.h>

#include "SimpleSupply.h"

SimpleSupply supply;


void setup() {
    Serial.begin(9600);
    supply.begin();

}

void loop() {
    u8g2.firstPage();
    do {
        u8g2.drawStr(0, 0, "Test");
//            ui.showVoltages(outVolt);
//            ui.showAmperage(outAmps);
    } while (u8g2.nextPage());
    delay(200);
}

void loop_() {
    time = millis();
    supply.loop();
    if (time > lastTime + refreshRate) {
        lastTime = time;
        supply.draw();
    }
}