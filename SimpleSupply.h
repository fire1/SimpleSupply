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
#define timeoutInterval 15000
#endif

//#define noDisplay

void static rotaryInterrupt();

void static buttonInterrupt();

volatile boolean btnStateReady = false;
const uint8_t pinTone = 11;
const uint8_t pinVoltPwm = 10;
const uint8_t pinMAmpInp = A3; // from 0 to 150ma
const uint8_t pinBAmpInp = A1; // from 0 to 150ma
const uint8_t pinVoltInp = A2;
const uint8_t pinTempInp = A0;
const uint8_t pinEncoderA = 4;
const uint8_t pinEncoderB = 3;
const uint8_t pinEncoderC = 2;
const uint8_t pinBlinker = LED_BUILTIN;
unsigned long time = 0, lastTime = 0;
volatile unsigned long btnLowTime = 0;
const uint16_t refreshRate = 850;
const uint16_t controlRate = 10;

const uint8_t lcdRow1 = 13;
const uint8_t lcdRow2 = 29;
const uint8_t *defFont = u8g2_font_crox3h_tf;
int refAmp = 0;

RotaryEncoder enc(pinEncoderA, pinEncoderB);
#ifndef noDisplay
U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C u8g2(U8G2_R2);
#endif

class SimpleSupply {
    enum class tones {
        none = 0, dir = 1, click = 2, hold = 3, low = 4, lows = 5, alarm = 6
    };
    enum class menus {
        main = 0, sub = 1, power = 2
    };

    char str[7];
    volatile uint8_t index;
    uint8_t soundIndex = 0;
    uint8_t cursor = 0;
    uint8_t pwmVolt = 0, lastPwm;
    uint16_t avrIndex = 0;
    uint16_t rawTemp = 0;
    int rawVolt, rawMAmp, rawBAmp;
    float setVolt = 0, setAmps = 1.5, outVolt, outAmps, nowTemp;
    tones play = tones::none;
    menus menu = menus::main;
    uint32_t avrMAmp = 0, avrVolt = 0, avrBAmp = 0;
    volatile unsigned long timeout = 0;
    unsigned long soundTime = 0;


public:
    void begin() {

        // Pin 9/10 timer setup
        TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
        Serial.println(F("Booting..."));
        pinMode(pinVoltInp, INPUT);
        pinMode(pinMAmpInp, INPUT);
        pinMode(pinBAmpInp, INPUT);
        pinMode(pinVoltPwm, OUTPUT);
        pinMode(pinTone, OUTPUT);
        pinMode(pinEncoderA, INPUT_PULLUP);
        pinMode(pinEncoderB, INPUT_PULLUP);
        pinMode(pinEncoderC, INPUT_PULLUP);
        enableInterrupt(pinEncoderA, rotaryInterrupt, CHANGE);
        enableInterrupt(pinEncoderB, rotaryInterrupt, CHANGE);
        enableInterrupt(pinEncoderC, buttonInterrupt, CHANGE);
#ifndef noDisplay
        u8g2.begin();
        u8g2.setFont(u8g2_font_crox3h_tf);
#endif
        analogWrite(pinVoltPwm, 0);
        tone(pinTone, 2000);
        delay(150);
        tone(pinTone, 2400);
        refAmp = analogRead(pinBAmpInp);
        delay(150);
        noTone(pinTone);
    }

    void loop() {
        this->parser();
        this->inject();
        this->editor();
        this->sounds();
        if (avrIndex > 15) {
            this->values();
            this->current();
        }
    }

    void draw() {
        digitalWrite(pinBlinker, LOW);
        this->temperature();
#ifndef noDisplay
        u8g2.firstPage();
        do {
            this->drawMain();
        } while (u8g2.nextPage());

#endif

        this->debug();
    }


private:
/**
 * Parsing live values
 */
    void parser() {
        uint32_t readVolt = 0, readMAmp = 0, readBAmp = 0;
        for (index = 0; index < 20; ++index) {
            readVolt += analogRead(pinVoltInp);
            readMAmp += analogRead(pinMAmpInp);
            readBAmp += analogRead(pinBAmpInp);
            delayMicroseconds(250);
        }
        avrVolt += readVolt / index;
        avrMAmp += readMAmp / index;
        avrBAmp += readBAmp / index;
        avrIndex++;
        if (lastPwm != pwmVolt) {
            // pwm 150 = 17V
            // max 160 = 18v
            // min 13 = 1.0v
            if (rawTemp > 280) {
                analogWrite(pinVoltPwm, pwmVolt);
                lastPwm = pwmVolt;
            } else {
                analogWrite(pinVoltPwm, 0);
            }
        }
    }

/**
 * Calculate average values
 */
    void values() {
        rawVolt = avrVolt / avrIndex;
        rawMAmp = avrMAmp / avrIndex;
        rawBAmp = avrBAmp / avrIndex;

        avrIndex = 0, avrMAmp = 0, avrBAmp = 0, avrVolt = 0;
        outVolt = map(rawVolt, 93, 906, 180, 1700) * 0.01;
//        if (rawMAmp < 620) {
//            outAmps = map(rawMAmp, 387, 633, 92, 150) * 0.001;
//        } else {
        outAmps = map(rawBAmp, 514, 420, 0, 3000) * 0.001;
//        }
    }

    void temperature() {
        rawTemp = analogRead(pinTempInp);
        nowTemp = map(rawTemp, 385, 270, 370, 500) * 0.1;
    }

    /**
     * Capture date from Serial
     */
    void inject() {
        if (Serial.available()) {
            String where = Serial.readStringUntil('=');

            //
            // Format v=<0-255>
            if (where == F("v")) {
                uint8_t volt = Serial.readStringUntil('\n').toInt();
                pwmVolt = volt;
                Serial.println();
                Serial.print(F("PWM: "));
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
        Serial.print(F(" mA set "));
        Serial.print(setAmps);
        Serial.print(F(" V raw "));
        Serial.print(rawVolt);
        Serial.print(F(" mA raw "));
        Serial.print(rawMAmp);
        Serial.print(F(" bA raw "));
        Serial.print(rawBAmp);
        Serial.print(F(" V out "));
        Serial.print(outVolt);
        Serial.print(F(" mA out "));
        Serial.print(outAmps);
        Serial.print(F(" PWM: "));
        Serial.print(pwmVolt);
        Serial.print(F(" CUR: "));
        Serial.print(cursor);
        Serial.print(F(" BTN: "));
        Serial.print(btnLowTime);
        Serial.print(F(" DIR: "));
        Serial.print((int8_t) enc.getDirection());
        Serial.print(F(" TMP: "));
        Serial.print(rawTemp);
        Serial.print(F(" / "));
        Serial.print(nowTemp);

    }

    void ping() {
        timeout = millis();
    }

    void blink() {
        digitalWrite(pinBlinker, HIGH);
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

        if (play == tones::low) {
            if (track(0)) sound(1300, 800);
            if (track(1)) mute();
        }

        if (play == tones::lows) {
            if (track(0)) sound(1600, 500);
            if (track(1)) silent(100);
            if (track(2)) sound(1600, 520);
            if (track(3)) mute();
        }

        if (play == tones::alarm) {
            if (track(0)) sound(1600, 500);
            if (track(1)) silent(100);
            if (track(2)) sound(1600, 500);
            if (track(3)) silent(100);
            if (track(4)) sound(1600, 500);
            if (track(4)) silent(100);
            if (track(6)) sound(1600, 500);
            if (track(7)) mute();
        }
    }

/**
 * Changes values of voltage and current
 * @param value
 * @param direction
 * @param rate
 * @return
 */
    uint8_t changeVoltValue(RotaryEncoder::Direction direction, uint8_t rate = 8) {
        int8_t dir = direction == RotaryEncoder::Direction::CLOCKWISE ? 1 : -1;
        if (pwmVolt > 58 && pwmVolt < 120)
            rate = 1 + rate;
        return (dir > 0) ? pwmVolt + rate : pwmVolt - rate;
    }

/**
 * Changes values of voltage and current
 * @param value
 * @param direction
 * @param rate
 * @return
 */
    float changeAmpsValue(RotaryEncoder::Direction direction, float rate = 1) {
        int8_t dir = direction == RotaryEncoder::Direction::CLOCKWISE ? 1 : -1;

        return (dir > 0) ? setAmps + rate : setAmps - rate;
    }

/**
 * Sets UI voltage to raw pwm
 * @param pwm
 */
    void setPwm(uint8_t pwm) {
        if (pwm > 165) {
            pwm = 250;
        }
        if (pwm >= 0 && pwm <= 250) {
            pwmVolt = pwm;
        }
    }

/**
  * Sets UI current and enables lowering in order to simulate current limiting
 * @param value
 */
    void setCurrent(float value) {
        setAmps = value;
    }


/**
 * Handles values change and menu
 */
    void editor() {
        this->clicker();
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
                        Serial.println((int8_t) direction);
                        this->setPwm(this->changeVoltValue(direction));
                        ping();
                    }
                    break;
                case 2:// fine edit of voltage
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setPwm(this->changeVoltValue(direction, 1));
                        ping();
                    }
                    break;


            }
            if (cursor > 2) {
                cursor = 1;
            }

        } else if (menu == menus::sub) {

            switch (cursor) {
                case 0:
                default://none
                    break;
                case 1:// fine edit of current (reverse order of voltage)
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setCurrent(this->changeAmpsValue(direction, 0.1));
                        ping();
                    }
                    break;

                case 2:// edit of current
                    if (direction != RotaryEncoder::Direction::NOROTATION) {
                        play = tones::dir;
                        this->setCurrent(this->changeAmpsValue(direction, 1));
                        ping();
                    }
            }
            if (cursor > 2) {
                cursor = 1;
            }
        }


        //
        // Triggers timeout
        if (time > (timeout + timeoutInterval) && timeout > 0) {
            cursor = 0;
            menu = menus::main;
            play = tones::lows;
            timeout = 0;
        }

    }


    void clicker() {
        if (btnStateReady && btnLowTime > 0 && btnLowTime < 800) {
            cursor++;
            play = tones::click;
            ping();
            blink();
            Serial.println();
            Serial.print(btnLowTime);
            Serial.println(F(" Click "));
            btnLowTime = 0;
            btnStateReady = false;
        } else if (btnStateReady && btnLowTime > 1000 && btnLowTime) {
            Serial.println();
            Serial.print(btnLowTime);
            Serial.println(F(" Hold "));
            menu = (menu == menus::sub) ? menus::main : menus::sub;
            play = tones::hold;
            ping();
            btnLowTime = 0;
            btnStateReady = false;
        }


    }


    void current() {
        if (setAmps > 0 && outAmps > setAmps) {
            pwmVolt--;
            return;
        }

        if (setAmps > 0 && outAmps < setAmps && outVolt < setVolt) {
            pwmVolt++;
        }
    }


/**
 * UI
 */
    void drawMain() {
        if (rawTemp > 280) {
            showVoltages(outVolt);
            if (menu == menus::sub) {
                showAmperage(setAmps);
            } else {
                showAmperage(outAmps);
            }
        } else {
            u8g2.setCursor(2, lcdRow1);
            u8g2.print(F("OVERHEAT"));
            blink();
        }
    }


    /**
     * Shows voltage on screen
     * @param voltage
     */
    void showVoltages(float voltage) {
#ifndef noDisplay
        u8g2.setCursor(2, lcdRow1);
        u8g2.print(F("V: "));
        sprintf(str, "%02d.%02d", (int) voltage, (int) (voltage * 100) % 100);
        u8g2.print(str);
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
        if (amperage < 0) {
            amperage = 0;
        }
        sprintf(str, "%01d.%03d", (int) amperage, (int) (amperage * 1000) % 1000);
        u8g2.print(str);
#endif

    }

};

void rotaryInterrupt() {
    enc.tick();
}


void buttonInterrupt() {
    if (digitalRead(pinEncoderC) == LOW && !btnStateReady && btnLowTime == 0) {
        btnLowTime = millis();
    }
    if (digitalRead(pinEncoderC) == HIGH && btnLowTime > 0 && !btnStateReady) {
        btnLowTime = millis() - btnLowTime;
        btnStateReady = true;
    }
    if (digitalRead(pinEncoderC) == LOW && millis() - btnLowTime > 1500 && !btnStateReady) {
        btnLowTime = millis() - btnLowTime;
        btnStateReady = true;
    }
}

#endif