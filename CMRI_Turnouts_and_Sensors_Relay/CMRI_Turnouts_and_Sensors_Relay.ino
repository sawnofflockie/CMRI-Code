// This sketch combines servo turnout control, sensor inputs and other outputs

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

// ----------------------------------------------
// -------------- Define constants --------------
// ----------------------------------------------

// CMRI Settings
#define CMRI_ADDR   1 //CMRI node address in JMRI
#define DE_PIN  2
#define CMRI_INPUTS     64
#define CMRI_OUTPUTS    128

#define BAUD_RATE   19200

// Servo frame rate must be 50Hz for analogue servos, can be up to 333Hz for digital servos
#define SERVO_FRAME_RATE    50

// The number of servos connected
#define NUMSERVOS   3

// The number of other outputs connected, e.g. lights, frogs etc
#define NUMOUTPUTS  2

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

#define CMRI_INPUT_RANGE_2_START 100

// ----------------------------------------------
// -------------- Set up hardware ---------------
// ----------------------------------------------

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //setup the board address - defaults to 0x40 if not specified

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

bool opLastState[NUMOUTPUTS] = {LOW, LOW};
bool opReqState[NUMOUTPUTS] = {LOW, LOW};

int outputType[NUMOUTPUTS] = {RELAY, LIGHT};

// ----------------------------------------------
// ----------------- FUNCTIONS ------------------
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

void processServos() {
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

void processSensors() {
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

void processOutputs() {
    // PROCESS OUTPUTS
    // Non servo outputs will start on bit 100, this will be address 1101 in CMRI, bit 101 will be 1102, etc.
    // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required

    for (int opNum = 0; opNum < NUMOUTPUTS; opNum++) {
        int outputPinNum = OUTPUT_RANGE_1_START + opNum;
        int cmriInput = CMRI_INPUT_RANGE_2_START + opNum;
        if (outputType[opNum] == LIGHT) {
            opReqState[opNum] = cmri.get_bit(cmriInput);
        }
        // bool opReqState = cmri.get_bit(100 + opNum);
        // Only process output if it has changed since the last time.
        if (opReqState[opNum] != opLastState[opNum]) {
            digitalWrite((outputPinNum), opReqState[opNum]);
            opLastState[opNum] = opReqState[opNum];
        }
    }
    // digitalWrite(46, cmri.get_bit(100)); //Frog Relay 1
    // digitalWrite(47, cmri.get_bit(101)); // Street lights
    //  digitalWrite(48, cmri.get_bit(102));
    //  etc...
    //  digitalWrite(67, cmri.get_bit(121));
    //  digitalWrite(68, cmri.get_bit(122));
    //  digitalWrite(69, cmri.get_bit(123));
}

// ----------------------------------------------

void setup() {
    int servoStart[NUMSERVOS] = {POINT_1_START, SIG_HEAD_1_START, SIG_HEAD_2_START};
    int servoEnd[NUMSERVOS] = {POINT_1_END, SIG_HEAD_1_END, SIG_HEAD_2_END};

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
    Serial.begin(BAUD_RATE); //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
    bus.begin(BAUD_RATE);

    // Initialize PCA9685 board
    pwm.begin();
    pwm.setPWMFreq(SERVO_FRAME_RATE);  // This is the maximum PWM frequency

    //SET THE THROW AND CLOSE VALUES FOR EACH SERVO BASED ON THE CALIBRATION PROCESS
    for (int servoNum = 0; servoNum<NUMSERVOS; servoNum++) {
        myServos[servoNum] = initialiseServo(myServos[servoNum], servoStart[servoNum], servoEnd[servoNum], servoNum);
    }

    // Initialize point motor virtual sensor to closed
    cmri.set_bit(8, HIGH);  //Bit 8 = address 1009 in JMRI, Virtual Sensor 9
}

// ----------------------------------------------

void loop(){
    cmri.process();

    processServos();
    processSensors();
    processOutputs();
}
