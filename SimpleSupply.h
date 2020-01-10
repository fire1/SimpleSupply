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


#include <U8g2lib.h>

#ifndef _U8G2LIB_HH

#include "../libraries/U8g2/src/U8g2lib.h"

#endif


#ifndef maxVoltage
#define maxVoltage 20
#endif

#ifndef timeoutInterval
#define timeoutInterval 1500
#endif

//#define noDisplay

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
const uint16_t refreshRate = 450;

const uint8_t lcdRow1 = 14;
const uint8_t lcdRow2 = 28;
const uint8_t *defFont = u8g2_font_crox3h_tf;


RotaryEncoder enc(pinEncoderA, pinEncoderB);
#ifndef noDisplay
U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C u8g2(U8G2_R2);
#endif

class SimpleSupply {
    enum class tones {
        none = 0, dir = 1, click = 2, hold = 3
    };
    enum class menus {
        main = 0, sub = 1, power = 2
    };

    uint8_t soundIndex = 0;
    uint8_t cursor = 0;
    uint8_t pwmVolt = 0, lastPwm;
    int rawVolt, rawAmps;
    float setVolt = 0, setAmps = 0, outVolt, outAmps;
    unsigned long timeout = 0;
    unsigned long soundTime = 0;
    tones play = tones::none;
    menus menu = menus::main;


public:
    void begin() {

        // Pin 9/10 timer setup
        TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
        Serial.println(F("Booting..."));
        pinMode(pinVoltInp, INPUT);
        pinMode(pinAmpsInp, INPUT);
        pinMode(pinVoltPwm, OUTPUT);
        pinMode(pinTone, OUTPUT);

        enableInterrupt(pinEncoderA, rotaryInterrupt, CHANGE);
        enableInterrupt(pinEncoderB, rotaryInterrupt, CHANGE);
        enableInterrupt(pinEncoderC, buttonInterrupt, CHANGE);
#ifndef noDisplay
        u8g2.begin();
        u8g2.setFont(defFont);
        u8g2.setPowerSave(0);

        u8g2.firstPage();
        do {
            u8g2.drawStr(0, 0, "Test");
        } while (u8g2.nextPage());
#endif
        tone(pinTone, 2000);
        delay(150);
        tone(pinTone, 2400);
        delay(150);
        noTone(pinTone);
    }

    void loop() {
        this->terminal();
        this->editor();
        this->power();
        this->sounds();
    }

    void draw() {
        this->current();


#ifndef noDisplay
        u8g2.firstPage();
        do {
            u8g2.drawStr(0, 0, "Test");
//            ui.showVoltages(outVolt);
//            ui.showAmperage(outAmps);
        } while (u8g2.nextPage());

#endif

        this->debug();
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

    void debug() {
        Serial.println();
        Serial.print(F(" V set "));
        Serial.print(setVolt);
        Serial.print(F(" A set "));
        Serial.print(setAmps);
        Serial.print(F(" V raw "));
        Serial.print(rawVolt);
        Serial.print(F(" A raw "));
        Serial.print(rawAmps);
        Serial.print(F(" V out "));
        Serial.print(outVolt);
        Serial.print(F(" A out "));
        Serial.print(outAmps);
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
        rawVolt = analogRead(pinVoltInp);
        rawAmps = analogRead(pinAmpsInp);
        if (lastPwm != pwmVolt) {
            analogWrite(pinVoltPwm, pwmVolt);
            lastPwm = pwmVolt;
        }
    }

/**
 * UI
 */
    class Display {
        char char3[4];

        /**
         * Converts float to lower decimal
         * @param value
         * @param output
         * @return
            */
        char displayFloat(float value, char *output) {
            if (value < -99) {
                value = -99;
            }
            int dig1 = int(value) * 10; // 210
            int dig2 = int((value * 10) - dig1);
            dig1 = dig1 / 10;
            if (dig2 < 0) {
                dig2 = dig2 * -1;
            }
            sprintf(output, "%02d.%1d", dig1, dig2);
        }

    public:

        /**
         * Shows voltage on screen
         * @param voltage
         */
        void showVoltages(float voltage) {
            displayFloat(voltage, char3);
#ifndef noDisplay
            u8g2.setCursor(2, lcdRow1);
            u8g2.print(F("V: "));
            u8g2.print(char3);
#endif;
        }

        /**
         * Shows amperage on screen
         * @param amperage
         */
        void showAmperage(float amperage) {
#ifndef noDisplay
            u8g2.setCursor(2, lcdRow2);
            u8g2.print(F("A: "));
#endif
            if (amperage == 0) {
#ifndef noDisplay
                u8g2.print("MAX");
#endif
            } else {
#ifndef noDisplay
                displayFloat(amperage, char3);
                u8g2.print(char3);
#endif
            }

        }
    };

    Display ui;
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