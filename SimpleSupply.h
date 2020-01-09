//
// Created by fire1 on 1/3/2020.
//
#ifndef SimpleSupply_h
#define SimpleSupply_h
const uint8_t pinVoltPwm = 10;
const uint8_t pinAmpsInp = A3;
const uint8_t pinVoltInp = A2;



void terminal() {
    if (Serial.available()) {
        String where = Serial.readStringUntil('=');

        if (where == F("v")) {
            uint8_t volt =Serial.readStringUntil('\n').toInt();
            analogWrite(pinVoltPwm, volt);
            Serial.println();
            Serial.print(F("VOLTAGE: "));
            Serial.print(volt);
            Serial.println();
        }


    }
}

#endif