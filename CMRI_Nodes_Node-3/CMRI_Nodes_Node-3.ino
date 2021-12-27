// Include libraries
#include <CMRI.h>
#include <Auto485.h>
#include <Adafruit_PWMServoDriver.h>

//#define ENABLE_DEBUG_OUTPUT

// CMRI Settings
#define CMRI_ADDR              3 //CMRI node address in JMRI
#define DE_PIN                 2

#define CMRI_INPUTS           24
#define CMRI_OUTPUTS          48

#define BAUD_RATE          19200
#define SERIAL_BAUD_RATE   19200

// Frame rate must be 50Hz for analogue servos, can be up to 333Hz for digital servos, but in this case it is for LEDs.
#define PWM_FRAME_RATE       120 // Can see 100Hz flicker so made it a bit faster. However if modelling fluorescent lights you might want 50Hz (i.e. mains frequency).
#define PWM_NORMAL_LEVEL    3072 // Max level is 4096, chose 3072 so there is room to go up as well as down.
#define NUM_PWM_OUTPUTS        3

#define PCA9685_ADDR        0x40

// -----------------------------
// LED street light states
// -----------------------------

#define OFF                    0 // Light completely extinguished.
#define ON                     1 // Light switched on.

// -----------------------------
// Day time states
// -----------------------------

#define DAYTIME                0
#define EVENING                1
#define NIGHT                  2

// -----------------------------
// Declare global variables
// -----------------------------

bool lightLevel1 = OFF;
bool lightLevel2 = OFF;

int level[NUM_PWM_OUTPUTS];

// int timeOfDay = DAYTIME;
int lastTimeOfDay = DAYTIME;

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR); //setup the board address - defaults to 0x40 if not specified

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 24 inputs and 48 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

// ----------------------------------------------
// ------------- FUNCTION PROTOTYPES ------------
// ----------------------------------------------
// void setup(void);
// void loop(void);
// void readFromCMRI(void);
// int setDayTime(void);
// void setLights(int timeOfDay);
// void illuminateLED(int level[NUM_PWM_OUTPUTS], int timeOfDay);
// ----------------------------------------------
// ----------------- FUNCTIONS ------------------
// ----------------------------------------------

void setup(void) {
    // Start the serial connection
    Serial.begin(SERIAL_BAUD_RATE); // Baud rate of the serial monitor used for debug output.
    bus.begin(BAUD_RATE); // Ensure this matches the baud rate in JMRI

    // Initialize PCA9685 board
    pwm.begin();
    pwm.setPWMFreq(PWM_FRAME_RATE);  // This is the maximum PWM frequency
}

void loop(void){
    int timeOfDay = DAYTIME;
    cmri.process();
    readFromCMRI();
    timeOfDay = setDayTime;
    setLights(timeOfDay);
    illuminateLED(level, timeOfDay);
}

void readFromCMRI(void) {
    lightLevel1 = cmri.get_bit(0);  // Bit 0 = address 3001 in JRMI
    lightLevel2 = cmri.get_bit(1);  // Bit 1 = address 3002 in JMRI
}

int setDayTime(void) {
    int timeOfDay = DAYTIME;
    if (lightLevel1 && lightLevel2) {
        timeOfDay = DAYTIME;
    } else if ((lightLevel1 && !lightLevel2) || (!lightLevel1 && lightLevel2)) {
        timeOfDay = EVENING;
    } else {
        timeOfDay = NIGHT;
    }
    return timeOfDay;
}

void setLights(int timeOfDay) {
    switch (timeOfDay) {
        case DAYTIME:
            if (level[DAYTIME] == OFF) {
                level[DAYTIME] = PWM_NORMAL_LEVEL;
                level[EVENING] = OFF;
                level[NIGHT] = OFF;
            }
        break;
        case EVENING:
            if (level[EVENING] == OFF) {
                level[DAYTIME] = OFF;
                level[EVENING] = PWM_NORMAL_LEVEL;
                level[NIGHT] = OFF;
            }
        break;
        case NIGHT:
            if (level[NIGHT] == OFF) {
                level[DAYTIME] = OFF;
                level[EVENING] = OFF;
                level[NIGHT] = PWM_NORMAL_LEVEL;
            }
        break;
    }
}

void illuminateLED(int level[NUM_PWM_OUTPUTS], int timeOfDay) {
    for (int pwmOutput = 0; pwmOutput < NUM_PWM_OUTPUTS; pwmOutput++) {
        if (lastTimeOfDay != timeOfDay) {
            pwm.writeMicroseconds(pwmOutput, level[pwmOutput]);
        }
    }
    lastTimeOfDay = timeOfDay;
}
