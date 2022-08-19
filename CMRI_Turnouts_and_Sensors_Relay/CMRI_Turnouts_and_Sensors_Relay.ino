// Pins 0, 1 and 2 are protected since these are used for communication
// Pin 13 is the Arduino LED pin and should not be used as an input, but can be used as an output for some applications
// Pins 20 and 21 are reserved for PWM servo control

// We will set the Arduino up to behave like a piece of CMRI hardware called a SUSIC with up to 64 slots
// Each slot has either a 24 or 32 bit input/output card
// 64 x 32 bit cards gives up to 2048 input/outputs!
// However, it's best to only set up the SUSIC with the required inputs/outputs to make the process more efficient.
// We will set cards 0 and 1 to be the sensor inputs (up to 64 inputs) and cards 2 - 5  to support 128 outputs

// ----------------------------------------------
// -------------- Include libraries -------------
// ----------------------------------------------

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CMRI.h>
#include <Auto485.h>
#include <TimerOne.h>

// ----------------------------------------------
// -------------- Define constants --------------
// ----------------------------------------------

// Uncomment the following line to enable debug output.
//#define ENABLE_DEBUG_OUTPUT

// CMRI Settings
#define CMRI_ADDR                     1 //CMRI node address in JMRI
#define DE_PIN                        2
#define CMRI_INPUTS                  64
#define CMRI_OUTPUTS                128
#define CMRI_INPUT_RANGE_2_START    100

#define BAUD_RATE           19200
#define SERIAL_BAUD_RATE    19200

// Servo frame rate must be 50Hz for analogue servos, can be up to 333Hz for digital servos
#define SERVO_FRAME_RATE     50
#define PWM_FRAME_RATE      120 // Can see 100Hz flicker so made it a bit faster. However if modelling fluorescent lights you might want 50Hz (i.e. mains frequency).
// #define PWM_FRAME_RATE       50 // For fluorescent lights.

// The number of servos connected
#define NUMSERVOS       3
// The number of other outputs connected, e.g. lights, frogs etc
#define NUMOUTPUTS      2
// The number of other PWM outputs that are not servos, e.g. street light LEDs. These have a separate board.
#define NUM_PWM_OUTPUTS 6
#define FIRST_ONE       0
#define LAST_ONE        (NUM_PWM_OUTPUTS - 1)

#define SIGNALBOX_LIGHTS    6
#define SIGBOX_FIRST        7
#define SIGBOX_LAST         (SIGBOX_FIRST + SIGNALBOX_LIGHTS - 1)

// The size of the step size used when signals are moving. The smaller the step size the slower the signal will move.
#define SIGSTEP     3
#define SIGDOWNSTEP 8

#define POINT_1         0
#define SIGNAL_HEAD_1   1
#define SIGNAL_HEAD_2   2

#define INPUT_RANGE_1_START   3
#define INPUT_RANGE_1_END    20
#define INPUT_RANGE_2_START  22
#define INPUT_RANGE_2_END    46
#define OUTPUT_RANGE_1_START 46
#define OUTPUT_RANGE_1_END   70

// SERVO MOTOR STATES
#define CLOSED        0
#define THROWN        1
#define CLOSING       2
#define THROWING      3
#define BOUNCING_UP   4
#define BOUNCING_DOWN 5

// Servo start and end points
#define POINT_1_START       1800
#define POINT_1_END         1200
#define SIG_HEAD_1_START    1670
#define SIG_HEAD_1_END      1160
#define SIG_HEAD_2_START    1820
#define SIG_HEAD_2_END      1330

// Servo output types
#define POINT_ELECTROFROG   0
#define POINT_INSULFROG     1
#define SIGNAL_HEAD         2

// Non-Point Output Types
#define RELAY   0
#define LIGHT   1

// Frog Relay Output Numbers
#define NO_RELAY        -1
#define FROG_RELAY_1     0

// ==================== Stuff for flickering LEDs ======================
#define PWM_NORMAL_LEVEL    3072 // Max level is 4096, chose 3072 so there is room to go up as well as down.
#define PWM_VARIANCE         384 // Max amount of variance up or down of LED level.

#define PWM_MIN_LEVEL   (PWM_NORMAL_LEVEL - PWM_VARIANCE)
#define PWM_MAX_LEVEL   (PWM_NORMAL_LEVEL + PWM_VARIANCE)

// -----------------------------
// All in milliseconds
// -----------------------------
#define FLICKER_MIN_TIME          100
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
// ================= End of stuff for flickering LEDs ==================

// -----------------------------
// Interrupt Period
// -----------------------------

#define INT_PERIOD         28572    // Number of micro seconds, so 5000 is once every 5 milli seconds (200 times a second), so the interrupt will activate 200 times a second.
                                    // The Arduino has a receive buffer of 64 characters, hence with 200 interrupts a second it can receive 12800 characters, or approx. 128000 bits.
                                    // i.e. more than 115200 baud would be capable of.

// ----------------------------------------------
// -------------- Set up hardware ---------------
// ----------------------------------------------

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Servo board
Adafruit_PWMServoDriver pwmLED = Adafruit_PWMServoDriver(0x41); // LED board

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 64 inputs and 128 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

// ----------------------------------------------
// ------------- Global Variables ---------------
// ----------------------------------------------

// Create tables to hold data about the servo positions
struct servo {
    int servoNum;
    int Throw;
    int Close;
    int state;
    int requiredState;
    int currentPosition;
    int upStep;
    int downStep;
    int bounce;
    int servoType; // Can be either POINT_ELECTROFROG, POINT_INSULFROG or SIGNAL
    int relayNum; // Number of the output for the associated frog relay
};

servo myServos[NUMSERVOS];

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
light SBLight[SIGNALBOX_LIGHTS];

bool opLastState[NUMOUTPUTS] = {LOW, LOW};
bool opReqState[NUMOUTPUTS] = {LOW, LOW};
bool changingState = false;

int outputType[NUMOUTPUTS] = {RELAY, LIGHT};

unsigned long currentTime; // The time, in milliseconds, of the current processing loop.

// ----------------------------------------------
// ------------- FUNCTION PROTOTYPES ------------
// ----------------------------------------------

void setup(void);
void loop(void);
void readFromCMRI(void);
void processOutputs(void);
void processSensors(void);
void processServos(void);
servo closeServo(servo);
servo throwServo(servo);
servo initialiseServo(servo, int, int, int);
void process_LED_outputs(bool);
void setup_wait_period(int);
void lightLED(int);
void outputLevel(int);

// ----------------------------------------------
// ----------------- FUNCTIONS ------------------
// ----------------------------------------------

void setup(void) {
    int servoStart[NUMSERVOS] = {POINT_1_START, SIG_HEAD_1_START, SIG_HEAD_2_START};
    int servoEnd[NUMSERVOS] = {POINT_1_END, SIG_HEAD_1_END, SIG_HEAD_2_END};

    randomSeed(analogRead(0)); // Reads noise on an unconnected pin to seed the random number generator.

    // SET PINS TO INPUT OR OUTPUT
    for (int inputPin=INPUT_RANGE_1_START; inputPin<INPUT_RANGE_1_END; inputPin++)  {
           pinMode(inputPin, INPUT_PULLUP);       // define sensor shield pins 3 to 19 as inputs
        }

    for (int inputPin=INPUT_RANGE_2_START; inputPin<INPUT_RANGE_2_END; inputPin++)  {
           pinMode(inputPin, INPUT_PULLUP);       // define sensor shield pins 22 to 45 as inputs
        }

    for (int outputPin=OUTPUT_RANGE_1_START; outputPin<OUTPUT_RANGE_1_END; outputPin++)  {
           pinMode(outputPin, OUTPUT);             // define sensor shield pins 46 to 69 as outputs
        }

    // Start the serial connection
    Serial.begin(SERIAL_BAUD_RATE); // Baud rate of the serial monitor used for debug output.
    bus.begin(BAUD_RATE); // Ensure this matches the baud rate in JMRI

    // Initialize PCA9685 boards
    pwm.begin();
    pwm.setPWMFreq(SERVO_FRAME_RATE);
    pwmLED.begin();
    pwmLED.setPWMFreq(PWM_FRAME_RATE);

    // Initialise the timer interrupt
    Timer1.initialize(INT_PERIOD);
    Timer1.attachInterrupt(readFromCMRI);

    //SET THE THROW AND CLOSE VALUES FOR EACH SERVO BASED ON THE CALIBRATION PROCESS
    for (int servoNum = 0; servoNum<NUMSERVOS; servoNum++) {
        myServos[servoNum] = initialiseServo(myServos[servoNum], servoStart[servoNum], servoEnd[servoNum], servoNum);
    }

    // Initialize point motor virtual sensor to closed
    cmri.set_bit(8, HIGH);  //Bit 8 = address 1009 in JMRI, Virtual Sensor 9

    for (int pwmOutput = FIRST_ONE; pwmOutput < NUM_PWM_OUTPUTS; pwmOutput++) {
        streetLight[pwmOutput].state = OFF;
        streetLight[pwmOutput].currentLevel = 0;
        streetLight[pwmOutput].lightType = GAS;
//        streetLight[pwmOutput].lightType = ELECTRIC;
        streetLight[pwmOutput].lastTime = 0;
        streetLight[pwmOutput].flickerDelay = random(FLICKER_MIN_TIME, FLICKER_MAX_TIME);
        streetLight[pwmOutput].waitPeriod = 0;
        streetLight[pwmOutput].waitStart = 0;
    }

    for (int pwmOutput = FIRST_ONE; pwmOutput < SIGNALBOX_LIGHTS; pwmOutput++) {
        SBLight[pwmOutput].state = OFF;
        SBLight[pwmOutput].currentLevel = 0;
        SBLight[pwmOutput].lightType = ELECTRIC;
        SBLight[pwmOutput].lastTime = 0;
        SBLight[pwmOutput].flickerDelay = 0;
        SBLight[pwmOutput].waitPeriod = 0;
        SBLight[pwmOutput].waitStart = 0;
    }
}

// ----------------------------------------------

void loop(){
    // cmri.process();

    processServos();
    processSensors();
    processOutputs();
}

// ----------------------------------------------

void readFromCMRI(void) {
    // Called by the timer interrupt.
    cmri.process();
}

// ----------------------------------------------

void processOutputs(void) {
    // PROCESS OUTPUTS
    // Non servo outputs will start on bit 100, this will be address 1101 in CMRI, bit 101 will be 1102, etc.

    for (int opNum = 0; opNum < NUMOUTPUTS; opNum++) {
        int outputPinNum = OUTPUT_RANGE_1_START + opNum;
        int cmriInput = CMRI_INPUT_RANGE_2_START + opNum;
        if (outputType[opNum] == LIGHT) {
            if (!changingState) {
                // only read the input if the system is not in the middle of changing the state of any of the lights.
                opReqState[opNum] = cmri.get_bit(cmriInput);
            }
            process_LED_outputs(opReqState[opNum]); // Process LED street lights every time through, in order to process flickers.
        }
        // Only process output if it has changed since the last time.
        if (opReqState[opNum] != opLastState[opNum]) {
            if (outputType[opNum] == RELAY) {
                digitalWrite((outputPinNum), opReqState[opNum]);
            }
            if (outputType[opNum] == LIGHT) {
                process_SB_lights(opReqState[opNum]);
            }
            opLastState[opNum] = opReqState[opNum];
        }
    }
    // digitalWrite(46, cmri.get_bit(100)); //Frog Relay 1
    // digitalWrite(47, cmri.get_bit(101)); // Street lights
}

// ----------------------------------------------

void processSensors(void) {
    // PROCESS SENSORS
    // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required
    // Ensure bit address matches pin, i.e. a sensor attached to pin 17 corresponds to bit 13 (because we've skipped pins 0, 1, 2 and 13) which is address 1014 for this CMRI node

    // Do not read 0, 1 or 2
    cmri.set_bit(0, !digitalRead(3));  //Bit 0 = address 1001 in JMRI, IR sensor 1
    cmri.set_bit(1, !digitalRead(4));  //Bit 1 = address 1002 in JMRI, IR Sensor 2
    cmri.set_bit(2, !digitalRead(5));  //Bit 2 = address 1003 in JMRI, IR Sensor 3
    cmri.set_bit(3, !digitalRead(6));  //Bit 3 = address 1004 in JMRI, Current Sensor 4
    cmri.set_bit(4, !digitalRead(7));  //Bit 4 = address 1005 in JMRI, Current Sensor 5
    cmri.set_bit(5, !digitalRead(8));  //Bit 5 = address 1006 in JMRI, Current Sensor 6
    cmri.set_bit(6, !digitalRead(9));  //Bit 6 = address 1007 in JMRI, Current Sensor 7
    cmri.set_bit(7, !digitalRead(10));  //Bit 7 = address 1008 in JMRI, Micro Switch Feedback Sensor 8
    cmri.set_bit(9, !digitalRead(11));  //Bit 9 = address 1010 in JMRI, LDR Sensor 10
    //cmri.set_bit(10, !digitalRead(14));  //Bit 9 = address 1011 in JMRI, Current Sensor 11
    //cmri.set_bit(11, !digitalRead(15));  //Bit 9 = address 1012 in JMRI, Current Sensor 12
    //etc.
    //Do not read 13
    //Do not read 20 or 21
}

// ----------------------------------------------

void processServos(void) {
    // PROCESS SERVOS
    // Assume servos start on bit 0 which corresponds to output address 1001
    for (int currentServo = 0; currentServo < NUMSERVOS; currentServo++) {
        myServos[currentServo].requiredState = cmri.get_bit(currentServo);
        if (myServos[currentServo].requiredState != myServos[currentServo].state) {
            // If requiredState is not equal to state then a movement is required,
            // requiredState should only ever be either CLOSED or THROWN, i.e. 0 or 1, as this comes from cmri.
            switch (myServos[currentServo].requiredState) {
                case THROWN:
                    myServos[currentServo] = throwServo(myServos[currentServo]);
                break;
                case CLOSED:
                    myServos[currentServo] = closeServo(myServos[currentServo]);
                break;
            }
            // If the relayNum has a value that is not NO_RELAY (-1) then it must be a point.
            if (myServos[currentServo].relayNum > NO_RELAY && myServos[currentServo].state <= 1) {
                // Only points have a frog relay and the state can only be CLOSED (0) or THROWN (1)
                cmri.set_bit(8, !myServos[currentServo].state);  //Bit 8 = address 1009 in JMRI, Virtual Feedback Sensor 9
                opReqState[myServos[currentServo].relayNum] = myServos[currentServo].state;   // Set bit state of frog relay
            }
            pwm.writeMicroseconds(currentServo, myServos[currentServo].currentPosition);
        }
    }
}

// ----------------------------------------------

servo closeServo(servo currentServo) {
    switch (currentServo.state) {
        case THROWN:
            currentServo.state = CLOSING;
        break;
        case CLOSING:
            // Check to see if movement is required.
            if (currentServo.currentPosition > currentServo.Close) {
                currentServo.currentPosition -= currentServo.downStep;
            } else {
                // Reached the end of travel, hence change the state.
                if (currentServo.servoType == SIGNAL_HEAD) {
                    currentServo.state = BOUNCING_UP;
                } else {
                    currentServo.state = CLOSED; // Points don't bounce
                }
            }
        break;
        case CLOSED:
            // Due to having a step size greater than 1 it would be possible to overshoot the end point.
            // Hence need to check for this and bring it back to the correct closed position.
            if (currentServo.currentPosition < currentServo.Close) {
                currentServo.currentPosition = currentServo.Close;
            }
        break;
        case BOUNCING_UP:
            // Check to see if movement is required.
            if (currentServo.currentPosition < currentServo.bounce) {
                currentServo.currentPosition += currentServo.downStep;
            } else {
                // Reached the end of travel, hence change the state.
                currentServo.state = BOUNCING_DOWN;
            }
        break;
        case BOUNCING_DOWN:
            // Check to see if movement is required.
            if (currentServo.currentPosition > currentServo.Close) {
                currentServo.currentPosition -= currentServo.downStep;
            } else {
                // Reached the end of travel, hence change the state.
                currentServo.state = CLOSED;
            }
        break;
    }

    return currentServo;
}

// ----------------------------------------------

servo throwServo(servo currentServo) {
    switch (currentServo.state) {
        case CLOSED:
            currentServo.state = THROWING;
        break;
        case THROWING:
            // Check to see if movement is required.
            if (currentServo.currentPosition < currentServo.Throw) {
                currentServo.currentPosition += currentServo.upStep;
            } else {
                // Reached the end of travel, hence change the state.
                currentServo.state = THROWN;
            }
        break;
        case THROWN:
            // Due to having a step size greater than 1 it would be possible to overshoot the end point.
            // Hence need to check for this and bring it back to the correct thrown position.
            if (currentServo.currentPosition > currentServo.Throw) {
                currentServo.currentPosition = currentServo.Throw;
            }
        break;
    }

    return currentServo;
}

// ----------------------------------------------

servo initialiseServo(servo currentServo, int Throw, int Close, int servoNum) {
    int throwRange;
    currentServo.servoNum = servoNum;
    currentServo.Throw = Throw;
    currentServo.Close = Close;
    currentServo.state = CLOSED;
    currentServo.currentPosition = Close;
    switch (currentServo.servoNum) {
        case POINT_1:
            currentServo.relayNum = FROG_RELAY_1;
            currentServo.servoType = POINT_ELECTROFROG;
            // don't bother with a step change here as the spring on the point stops it moving slowly.
            currentServo.upStep = Throw - Close;
            currentServo.downStep = Throw - Close;
        break;
        case SIGNAL_HEAD_1:
        case SIGNAL_HEAD_2:
            currentServo.relayNum = NO_RELAY;
            currentServo.upStep = SIGSTEP;
            currentServo.downStep = SIGDOWNSTEP;
            currentServo.servoType = SIGNAL_HEAD;
        break;
    }
    throwRange = Throw - Close;
    currentServo.bounce = (throwRange / 3) + Close; // Increase the divisor to decrease the amplitude of the bounce.
    pwm.writeMicroseconds(currentServo.servoNum, currentServo.Close);

    return currentServo;
}

// ----------------------------------------------

void process_LED_outputs(bool requiredState) {
    currentTime = millis();
    for (int pwmOutput = FIRST_ONE; pwmOutput < NUM_PWM_OUTPUTS; pwmOutput++) {
        switch (streetLight[pwmOutput].state) {
            case OFF:
                if (requiredState == ON) {
                    streetLight[pwmOutput].state = WAIT_ON;
                    setup_wait_period(pwmOutput);
                    if (pwmOutput == FIRST_ONE) {
                        changingState = true;
                    }
                } else if (pwmOutput == LAST_ONE) {
                    changingState = false;
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
                    if (pwmOutput == FIRST_ONE) {
                        changingState = true;
                    }
                } else if (pwmOutput == LAST_ONE) {
                    changingState = false;
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

// ----------------------------------------------

void process_SB_lights(bool requiredState) {
    for (int pwmOutput = FIRST_ONE; pwmOutput < SIGNALBOX_LIGHTS; pwmOutput++) {
        switch (SBLight[pwmOutput].state) {
            case OFF:
                if (requiredState == ON) {
                    SBLight[pwmOutput].state = ON;
                    SBLight[pwmOutput].currentLevel = PWM_NORMAL_LEVEL;
                }
            break;
            case ON:
                if (requiredState == OFF) {
                    SBLight[pwmOutput].state = ON;
                    SBLight[pwmOutput].currentLevel = OFF;
                }
            break;
        }
        pwmLED.writeMicroseconds((pwmOutput + SIGBOX_FIRST), SBLight[pwmOutput].currentLevel);
    }
}

// ----------------------------------------------

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

// ----------------------------------------------

void lightLED(int pwmOutput) {
    if ((currentTime - streetLight[pwmOutput].lastTime) > streetLight[pwmOutput].flickerDelay) {
        outputLevel(pwmOutput);
        pwmLED.writeMicroseconds(pwmOutput, streetLight[pwmOutput].currentLevel);
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

// ----------------------------------------------

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

// ----------------------------------------------