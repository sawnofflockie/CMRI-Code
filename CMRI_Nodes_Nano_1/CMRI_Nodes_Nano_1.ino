// Include libraries
#include <CMRI.h>
#include <Auto485.h>
#include <Adafruit_PWMServoDriver.h>

// CMRI Settings
#define CMRI_ADDR   2 //CMRI node address in JMRI
#define DE_PIN      2

#define CMRI_INPUTS     24
#define CMRI_OUTPUTS    48

#define BAUD_RATE           19200
#define SERIAL_BAUD_RATE    19200

// -----------------------------
#define INPUT_RANGE_START    3
#define INPUT_RANGE_END      6
#define OUTPUT_RANGE_START   7
#define OUTPUT_RANGE_END    13 // Pin 13 corresponds to the Arduino on-board LED
// -----------------------------

#define NUM_PWM_OUTPUTS   3

// Servo frame rate must be 50Hz for analogue servos, can be up to 333Hz for digital servos, but in this case it is for LEDs.
#define PWM_FRAME_RATE    120 // Can see 100Hz flicker, so made it a bit faster.

#define PWM_NORMAL_LEVEL    3072 // Max level is 4096, chose 3072 so there is room to go up as well as down.
#define PWM_VARIANCE         512 // Max amount of variance up or down of LED level.

#define PWM_MIN_LEVEL   (PWM_NORMAL_LEVEL - PWM_VARIANCE)
#define PWM_MAX_LEVEL   (PWM_NORMAL_LEVEL + PWM_VARIANCE)

// -------------------
// All in milliseconds
// -------------------
#define FLICKER_MIN_TIME           75
#define FLICKER_MAX_TIME          350

#define ELECTRIC_WAIT_PERIOD     5000
#define MIN_WAIT_PERIOD         20000
#define MAX_WAIT_PERIOD         30000

#define LIGHT_LEVEL_STEP          100
// -------------------

// -----------------------------
// LED street light states
// -----------------------------
#define OFF         0 // Light completely extinguished and no change of state imminent.
#define ON          1 // Light switched on with perhaps some random variations in intensity.
#define WAIT_ON     2 // A change of state to ON has been requested, but the light is waiting for its turn to go on.
#define GOING_ON    3 // A change to ON is now being actioned and the light is ramping up to operational intensity.
#define GOING_OFF   4 // A change to OFF is now being actioned and the light is ramping down to zero intensity.
#define WAIT_OFF    5 // A change of state to OFF has been requested, but the light is waiting for its turn to go off.

// -----------------------------
// LED street light types
// -----------------------------
#define ELECTRIC    0
#define GAS         1

// Create structure to hold data about the lights
struct light {
    int state; // Can be OFF, WAIT_ON, GOING_ON, ON, WAIT_OFF, GOING_OFF.
    int currentLevel; // Holds the light intensity aka the pulse width.
    int lightType; // Can be either ELECTRIC (0) or GAS (1)
    unsigned long lastTime; // The time, in milliseconds, since the last processing loop for a given light.
    unsigned long flickerDelay; // The time to wait before the next random increase or decrease in light intensity.
    unsigned long waitPeriod; // The amount of time to wait before starting to switch on or switch off a light.
    unsigned long waitStart; // The time at which to start the process of switching a light on or off.
};

light streetLight[NUM_PWM_OUTPUTS];

bool requiredState = OFF;
unsigned long currentTime; // The time, in milliseconds, of the current processing loop.

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //setup the board address - defaults to 0x40 if not specified

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 24 inputs and 48 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

void setup() {

    // SET PINS TO INPUT OR OUTPUT
    // However in the current setup there are no inputs and all outputs are via the PCA9685 board (pwm).
    // for (int i=INPUT_RANGE_START; i<=INPUT_RANGE_END; i++) {
    //        pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 7 as inputs - 5 inputs.
    // }
    // for (int i=OUTPUT_RANGE_START; i<=OUTPUT_RANGE_END; i++) {
    //        pinMode(i, OUTPUT);      // define sensor shield pins 8 to 13 as outputs - 5 outputs, plus pin 13 which is the built-in LED.
    // }

    randomSeed(analogRead(0));

    for (int pwmOutput = 0; pwmOutput < NUM_PWM_OUTPUTS; pwmOutput++) {
        streetLight[pwmOutput].state = OFF;
        streetLight[pwmOutput].currentLevel = 0;
        streetLight[pwmOutput].lightType = GAS;
//        streetLight[pwmOutput].lightType = ELECTRIC;
        streetLight[pwmOutput].lastTime = 0;
        streetLight[pwmOutput].flickerDelay = random(FLICKER_MIN_TIME, FLICKER_MAX_TIME);
        streetLight[pwmOutput].waitPeriod = 0;
        streetLight[pwmOutput].waitStart = 0;
    }
    // Start the serial connection
    Serial.begin(SERIAL_BAUD_RATE); //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
    bus.begin(BAUD_RATE);

    // Initialize PCA9685 board
    pwm.begin();
    pwm.setPWMFreq(PWM_FRAME_RATE);  // This is the maximum PWM frequency
}

void loop(){
    cmri.process();

    // PROCESS SENSORS
    // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required

    // Do not read 0, 1 or 2
    // cmri.set_bit(0, !digitalRead(3));  //Bit 0 = address 2001 in JMRI, Light sensor 1
    // cmri.set_bit(1, !digitalRead(4));  //Bit 1 = address 2002 in JMRI, Light sensor 2

    // PROCESS OUTPUTS
    process_outputs();
}

void process_outputs() {
    currentTime = millis();
    requiredState = cmri.get_bit(0); //Bit 0 = address 2001 in JMRI, LED output 1
    for (int pwmOutput = 0; pwmOutput < NUM_PWM_OUTPUTS; pwmOutput++) {
        switch (streetLight[pwmOutput].state) {
            case OFF:
                if (requiredState == ON) {
                    streetLight[pwmOutput].state = WAIT_ON;
                    setup_wait_period(pwmOutput);
                }
            break;
            case WAIT_ON:
                if (currentTime > streetLight[pwmOutput].waitStart + streetLight[pwmOutput].waitPeriod) {
                    streetLight[pwmOutput].state = GOING_ON;
                }
            break;
            case ON:
                if (requiredState == OFF) {
                    streetLight[pwmOutput].state = WAIT_OFF;
                    setup_wait_period(pwmOutput);
                }
            break;
            case WAIT_OFF:
                if (currentTime > streetLight[pwmOutput].waitStart + streetLight[pwmOutput].waitPeriod) {
                    streetLight[pwmOutput].state = GOING_OFF;
                }
            case GOING_ON:
            case GOING_OFF:
            break;
        }
        lightLED(pwmOutput);
    }
}

void setup_wait_period(int pwmOutput) {
        switch (streetLight[pwmOutput].lightType) {
            case ELECTRIC:
                // Real electric lights come on randomly due to slight variations in their light sensors.
                streetLight[pwmOutput].waitPeriod = random(ELECTRIC_WAIT_PERIOD);
            break;
            case GAS:
                // Want the gas lights to come on one after another with a suitable random delay.
                // This is to simulate a bloke lighting one light then walking, ladder in hand, to the next and so on.
                streetLight[pwmOutput].waitPeriod = random(MIN_WAIT_PERIOD, MAX_WAIT_PERIOD) * (unsigned long)pwmOutput;
            break;
        }
        streetLight[pwmOutput].waitStart = currentTime;
}

void lightLED(int pwmOutput) {
    if ((currentTime - streetLight[pwmOutput].lastTime) > streetLight[pwmOutput].flickerDelay) {
        outputLevel(pwmOutput);
        pwm.writeMicroseconds(pwmOutput, streetLight[pwmOutput].currentLevel);
        // Don't want a flicker if the light is electric.
        // Using a switch instead now as, although it makes the code slightly bigger, it is slightly faster and easier to follow.
        switch (streetLight[pwmOutput].lightType) {
            case ELECTRIC:
                streetLight[pwmOutput].flickerDelay = 0;
            break;
            case GAS:
                streetLight[pwmOutput].flickerDelay = random(FLICKER_MIN_TIME, FLICKER_MAX_TIME);
            break;
        }
        streetLight[pwmOutput].lastTime = currentTime;
    }
}

void outputLevel(int pwmOutput) {
    switch (streetLight[pwmOutput].state) {
        case OFF:
        case WAIT_ON:
            streetLight[pwmOutput].currentLevel = OFF;
        break;
        case GOING_ON:
            if (streetLight[pwmOutput].currentLevel < PWM_NORMAL_LEVEL && streetLight[pwmOutput].lightType == GAS) {
                // If it's a gas light simulate the tap being switched on.
                streetLight[pwmOutput].currentLevel += LIGHT_LEVEL_STEP;
            } else {
                // If electric light, just go on.
                streetLight[pwmOutput].state = ON;
            }
        break;
        case ON:
        case WAIT_OFF:
            // There was no discernable benefit in terms of speed when using the switch statement, due to smaller data types in use this time.
            // However the code was smaller and it's easier to follow, so staying with the switch.
            switch (streetLight[pwmOutput].lightType) {
                case ELECTRIC:
                    streetLight[pwmOutput].currentLevel = PWM_NORMAL_LEVEL;
                break;
                case GAS:
                    streetLight[pwmOutput].currentLevel = random(PWM_MIN_LEVEL, PWM_MAX_LEVEL);
                break;
            }
        break;
        case GOING_OFF:
            if (streetLight[pwmOutput].currentLevel > OFF && streetLight[pwmOutput].lightType == GAS) {
                // If it's a gas light simulate the tap being switched off.
                streetLight[pwmOutput].currentLevel -= LIGHT_LEVEL_STEP;
            } else {
                // If electric light, just go off.
                streetLight[pwmOutput].state = OFF;
            }
        break;
    }
}
