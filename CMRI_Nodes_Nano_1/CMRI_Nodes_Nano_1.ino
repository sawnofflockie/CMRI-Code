// Include libraries
#include <CMRI.h>
#include <Auto485.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>

//#define ENABLE_DEBUG_OUTPUT

// CMRI Settings
#define CMRI_ADDR   2 //CMRI node address in JMRI
#define DE_PIN      2

#define CMRI_INPUTS     24
#define CMRI_OUTPUTS    48

#define BAUD_RATE           115200
#define SERIAL_BAUD_RATE    19200

#define NUM_PWM_OUTPUTS   3

// Servo frame rate must be 50Hz for analogue servos, can be up to 333Hz for digital servos, but in this case it is for LEDs.
#define PWM_FRAME_RATE    120 // Can see 100Hz flicker so made it a bit faster. However if modelling fluorescent lights you might want 50Hz (i.e. mains frequency).

#define PWM_NORMAL_LEVEL    3072 // Max level is 4096, chose 3072 so there is room to go up as well as down.
#define PWM_VARIANCE         512 // Max amount of variance up or down of LED level.

#define PWM_MIN_LEVEL   (PWM_NORMAL_LEVEL - PWM_VARIANCE)
#define PWM_MAX_LEVEL   (PWM_NORMAL_LEVEL + PWM_VARIANCE)

// -----------------------------
// All in milliseconds
// -----------------------------
#define FLICKER_MIN_TIME           75
#define FLICKER_MAX_TIME          350

#define ELECTRIC_WAIT_PERIOD    30000
#define MIN_WAIT_PERIOD         20000
#define MAX_WAIT_PERIOD         30000

#define LIGHT_LEVEL_STEP          100
// -----------------------------

// -----------------------------
// LED street light states
// -----------------------------
#define OFF         0 // Light completely extinguished and no change of state imminent.
#define ON          1 // Light switched on with perhaps some random variations in intensity, but no change of state imminent.
#define WAIT_ON     2 // A change of state to ON has been requested, but the light is waiting for its turn to go on.
#define GOING_ON    3 // A change to ON is now being actioned and the light is ramping up to operational intensity.
#define GOING_OFF   4 // A change to OFF is now being actioned and the light is ramping down to zero intensity.
#define WAIT_OFF    5 // A change of state to OFF has been requested, but the light is waiting for its turn to go off.

// -----------------------------
// LED street light types
// -----------------------------
#define ELECTRIC    0
#define GAS         1

// -----------------------------
// Interrupt Period
// -----------------------------

#define INT_PERIOD          5000    // Number of micro seconds, so 5000 is once every 5 milli seconds (200 times a second), so the interrupt will activate 200 times a second.
                                    // The Arduino has a receive buffer of 64 characters, hence with 200 interrupts a second it can receive 12800 characters, or approx. 128000 bits.
                                    // i.e. more than 115200 baud would be capable of.

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

volatile bool requiredState = OFF;
volatile bool internalReqState = OFF;
bool currentInternalState = OFF;
unsigned long currentTime; // The time, in milliseconds, of the current processing loop.

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //setup the board address - defaults to 0x40 if not specified
// Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second board, address 0x41. Needs pads on board soldering to select address.

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 24 inputs and 48 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

// ----------------------------------------------
// ------------- FUNCTION PROTOTYPES ------------
// ----------------------------------------------

void setup(void);
void loop(void);
void readFromCMRI(void);
void process_outputs(void);
void processInternalLED(void);
void setup_wait_period(int pwmOutput);
void lightLED(int pwmOutput);
void outputLevel(int pwmOutput);

// ----------------------------------------------
// ----------------- FUNCTIONS ------------------
// ----------------------------------------------

void setup(void) {
    randomSeed(analogRead(0)); // Reads noise on an unconnected pin to seed the random number generator.

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
    // Start the serial connections
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.begin(SERIAL_BAUD_RATE); // Baud rate of the serial monitor used for debug output.
#endif
    bus.begin(BAUD_RATE); // Ensure this matches the baud rate in JMRI

    // Initialize PCA9685 board
    pwm.begin();
    pwm.setPWMFreq(PWM_FRAME_RATE);  // This is the maximum PWM frequency

    // Initialise the timer interrupt
    Timer1.initialize(INT_PERIOD);
    Timer1.attachInterrupt(readFromCMRI);
}

void loop(void){
    process_outputs();
}

void readFromCMRI(void) {
    // Called via timer1 interrupt.
    cmri.process();
    requiredState = cmri.get_bit(0); //Bit 0 = address 2001 in JMRI, LED output 1
    internalReqState = cmri.get_bit(1); //Bit 1 = address 2002 in JMRI, LED output 2
}

void process_outputs(void) {
    currentTime = millis();
    processInternalLED();
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
            break;
            case GOING_ON:
            case GOING_OFF:
            break;
        }
        lightLED(pwmOutput);
#ifdef ENABLE_DEBUG_OUTPUT
        Serial.print("pwmOutput = ");
        Serial.print(pwmOutput);
        Serial.print(", state= ");
        Serial.println(streetLight[pwmOutput].state);
#endif
    }
}

void processInternalLED(void) {
    if (currentInternalState != internalReqState) {
        currentInternalState = internalReqState;
        digitalWrite(13, internalReqState);
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
            // There was no discernible benefit in terms of speed when using the switch statement, due to smaller data types in use this time.
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
