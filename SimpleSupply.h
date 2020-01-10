//
// Created by fire1 on 1/3/2020.
//
#ifndef SimpleSupply_h
#define SimpleSupply_h

#include <RotaryEncoder.h>

#ifndef RotaryEncoder_h

#include "../libraries/RotaryEncoder/RotaryEncoder.h"

#endif

#include <EnableInterrupt.h>

#ifndef EnableInterrupt_h

#include "../libraries/EnableInterrupt/EnableInterrupt.h"

#endif

#ifndef maxVoltage
#define maxVoltage 20
#endif

#ifndef timeoutInterval
#define timeoutInterval 1500
#endif

void static rotaryInterrupt();

void static buttonInterrupt();

const uint8_t pinTone = 11;
const uint8_t pinVoltPwm = 10;
const uint8_t pinAmpsInp = A3;
const uint8_t pinVoltInp = A2;
const uint8_t pinEncoderA = 3;
const uint8_t pinEncoderB = 4;
const uint8_t pinEncoderC = 2;
unsigned long time = 0, lastTime = 0, btnLowTime = 0;
const uint16_t refreshRate = 250;


RotaryEncoder enc(pinEncoderA, pinEncoderB);

class SimpleSupply {
    enum class tones {
        none = 0, dir = 1, click = 2, hold = 3
    };
    enum class menus {
        main = 0, sub = 1, power = 2
    };
    tones play = tones::none;
    menus menu = menus::main;
    uint8_t soundIndex = 0;
    uint8_t cursor = 0;
    uint8_t pwmVolt = 0, lastPwm;
    float setVolt = 0, setAmps = 0, outVolt, outAmps;
    unsigned long timeout = 0;
    unsigned long soundTime = 0;


public:
    void begin() {

        pinMode(pinVoltPwm, OUTPUT);
        // Pin 9/10 timer setup
        TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
        Serial.println(F("Booting..."));
        pinMode(pinVoltInp, INPUT);
        pinMode(pinAmpsInp, INPUT);

        enableInterrupt(pinEncoderA, rotaryInterrupt, CHANGE);
        enableInterrupt(pinEncoderB, rotaryInterrupt, CHANGE);
        enableInterrupt(pinEncoderC, buttonInterrupt, CHANGE);
    }

    void loop() {
        this->terminal();
        this->editor();
        this->power();
        this->sounds();
    }

    void draw() {
        current();
    }


private:
    /**
     * Capture date from Serial
     */
    void terminal() {
        if (Serial.available()) {
            String where = Serial.readStringUntil('=');

            //
            // Format v=<0-255>
            if (where == F("v")) {
                uint8_t volt = Serial.readStringUntil('\n').toInt();
                analogWrite(pinVoltPwm, volt);
                Serial.println();
                Serial.print(F("SET VOLTAGE: "));
                Serial.print(volt);
                Serial.println();
            }

            //
            // Format a=<0-255>
            if (where == F("a")) {
                uint8_t volt = Serial.readStringUntil('\n').toInt();
                analogWrite(pinVoltPwm, volt);
                Serial.println();
                Serial.print(F("SET CURRENT: "));
                Serial.print(volt);
                Serial.println();
            }


        }
    }

    void ping() {
        timeout = millis();
    }


    void sound(int hertz, uint16_t interval = 50) {
        tone(pinTone, 2200);
        soundIndex++;
        soundTime = time + interval;
    }

    void mute() {
        soundIndex = 0;
        soundTime = 0;
        play = tones::none;
        noTone(pinTone);
    }

    void silent(uint16_t interval = 50) {
        noTone(pinTone);
        soundIndex++;
        soundTime = time + interval;
    }

    boolean track(uint8_t index) {
        return soundTime < time && soundIndex == index;
    }


    void sounds() {
        if (play == tones::dir) {
            if (track(0)) sound(2200, 25);
            if (track(1)) mute();
        }

        if (play == tones::click) {
            if (track(0)) sound(2000);
            if (track(1)) silent();
            if (track(2)) sound(2000);
            if (track(3)) mute();
        }

        if (play == tones::hold) {
            if (track(0)) sound(2000, 200);
            if (track(1)) silent(100);
            if (track(2)) sound(20400, 220);
            if (track(3)) mute();
        }
    }

/**
 * Changes values of voltage and current
 * @param value
 * @param direction
 * @param rate
 * @return
 */
    float changeValue(float value, int8_t direction, double rate = 1) {
        return (direction > 0) ? value + rate : value - rate;
    }

/**
 * Sets UI voltage to raw pwm
 * @param value
 */
    void setVoltage(float value) {
        if (value > maxVoltage) {
            value = 20;
        }
        if (value >= 0 && value <= maxVoltage) {
            setVolt = value;
            if (value == maxVoltage) {
                pwmVolt = 255;
            }
            pwmVolt = map(value * 10, 10, 205, 15, 167);
        }
    }

/**
  * Sets UI current and enables lowering in order to simulate current limiting
 * @param value
 */
    void setCurrent(float value) {

    }

/**
 * Handles values change and menu
 */
    void editor() {
        if (btnLowTime > 0 && btnLowTime < 100) {
            cursor++;
            ping();
        }
        if (btnLowTime > 300 && btnLowTime < 1000) {
            menu = menus::sub;
            ping();
        }

        RotaryEncoder::Direction direction = enc.getDirection();
        //
        // Main menu (Home)
        if (menu == menus::main) {
            switch (cursor) {
                case 0:
                default://none
                    break;
                case 1: // edit voltage
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setVoltage(this->changeValue(setVolt, (int) direction));
                        ping();
                    }
                    break;
                case 2:// fine edit of voltage
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setVoltage(this->changeValue(setVolt, (int) direction, 0.010));
                        ping();
                    }
                    break;

                case 4:// fine edit of current (reverse order of voltage)
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setCurrent(this->changeValue(setVolt, (int) direction));
                        ping();
                    }
                    break;

                case 3:// edit of current
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setCurrent(this->changeValue(setVolt, (int) direction, 0.010));
                        ping();
                    }
                    break;
            }

        } else if (menu == menus::sub) {

        }


        //
        // Triggers timeout
        if (time > (timeout + timeoutInterval)) {
            cursor = 0;
            menu = menus::main;
        }

    }


    void current() {
        if (setAmps > 0 && outAmps > setAmps) {
            pwmVolt--;
        }

        if (setAmps > 0 && outAmps < setAmps && outVolt < setVolt) {
            pwmVolt++;
        }
    }

/**
 * Controlling
 */
    void power() {


        if (lastPwm != pwmVolt) {
            analogWrite(pinVoltPwm, pwmVolt);
            lastPwm = pwmVolt;
        }
    }

};

void rotaryInterrupt() {
    enc.tick();
}

void buttonInterrupt() {
    if (digitalRead(pinEncoderC) == LOW) {
        btnLowTime = millis();
    } else {
        btnLowTime = millis() - btnLowTime;
    }
}

#endif