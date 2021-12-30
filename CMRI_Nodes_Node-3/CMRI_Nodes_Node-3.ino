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
#define PWM_MAX_LEVEL       4096
#define NUM_PWM_OUTPUTS        4 // Number of LEDs indicating time of day; DAYTIME, EVENING, NIGHT, DAWN

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
#define DAWN                   3

// -----------------------------
// Declare global variables
// -----------------------------

bool lightLevel1 = OFF;
bool lightLevel2 = OFF;

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
void setup(void);
void loop(void);
void readFromCMRI(void);
int setDayTime(int lastDayTime);
int illuminateLED(int timeOfDay, int lastTimeOfDay);
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
    timeOfDay = setDayTime(lastTimeOfDay);
    lastTimeOfDay = illuminateLED(timeOfDay, lastTimeOfDay);
}

void readFromCMRI(void) {
    lightLevel1 = cmri.get_bit(0);  // Bit 0 = address 3001 in JRMI
    lightLevel2 = cmri.get_bit(1);  // Bit 1 = address 3002 in JMRI
}

int setDayTime(int lastTimeOfDay) {
    int timeOfDay;
    if (lightLevel2) {
        if (lightLevel1) {
            timeOfDay = DAYTIME;
        } else {
            switch (lastTimeOfDay) {
                case DAYTIME:
                    timeOfDay = EVENING;
                break;
                case NIGHT:
                    timeOfDay = DAWN;
                break;
                default:
                    timeOfDay = lastTimeOfDay;
                break;
            }
        }
    } else {
        timeOfDay = NIGHT;
    }
    return timeOfDay;
}

int illuminateLED(int timeOfDay, int lastTimeOfDay) {
    if (lastTimeOfDay != timeOfDay) {
        for (int pwmOutput = 0; pwmOutput < NUM_PWM_OUTPUTS; pwmOutput++) {
            int level;
            if (pwmOutput == timeOfDay) {
                level = PWM_NORMAL_LEVEL;
            }
            else {
                level = OFF;
            }
            pwm.writeMicroseconds(pwmOutput, level);
        }
        lastTimeOfDay = timeOfDay;
    }
    return lastTimeOfDay;
}
