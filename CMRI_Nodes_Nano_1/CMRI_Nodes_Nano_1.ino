// Include libraries
#include <CMRI.h>
#include <Auto485.h>
#include <Adafruit_PWMServoDriver.h>

// CMRI Settings
#define CMRI_ADDR 2 //CMRI node address in JMRI
#define DE_PIN 2

#define CMRI_INPUTS 24
#define CMRI_OUTPUTS 48

//#define BAUD_RATE 19200
#define BAUD_RATE 28800

#define NUMPWMOUTPUTS   2

#define PWM_NORMAL_LEVEL    3072 // Max level is 4096, chose 3072 so there is room to go up as well as down.
#define PWM_VARIANCE        1024 // Max amount of variance up or down of LED level.

// Servo frame rate must be 50Hz for analogue servos, can be up to 333Hz for digital servos, but in this case it is for LEDs.
#define PWM_FRAME_RATE    120 // Can see 100Hz flicker, so made it a bit faster.

// Both in milliseconds
#define FLICKER_MIN_TIME    75
#define FLICKER_MAX_TIME    350

// -----------------------------
#define INPUT_RANGE_START    3
#define INPUT_RANGE_END      6
#define OUTPUT_RANGE_START   7
#define OUTPUT_RANGE_END    13 // Pin 13 corresponds to the Arduino on-board LED
// -----------------------------

bool requiredState = LOW;
unsigned long currentTime;
unsigned long lastTime[NUMPWMOUTPUTS];
unsigned long flickerDelay[NUMPWMOUTPUTS];

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //setup the board address - defaults to 0x40 if not specified

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 24 inputs and 48 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

int outputLevel(int level) {
    if (requiredState == HIGH) {
        level = PWM_NORMAL_LEVEL - PWM_VARIANCE + random(PWM_VARIANCE * 2);
    } else {
        level = 0;
    }
    return level;
}

void lightLED() {
    int level = 0;
    currentTime = millis();
    for (int pwmOutput = 0; pwmOutput < NUMPWMOUTPUTS; pwmOutput++) {
        if ((currentTime - lastTime[pwmOutput]) > flickerDelay[pwmOutput]) {
            level = outputLevel(level);
            pwm.writeMicroseconds(pwmOutput, level);
            flickerDelay[pwmOutput] = random(FLICKER_MIN_TIME, FLICKER_MAX_TIME);
            lastTime[pwmOutput] = currentTime;
        }
    }
}

void process_outputs() {
    requiredState = cmri.get_bit(0); //Bit 0 = address 2001 in JMRI, LED output 1
    lightLED();
}

void setup() {

    // SET PINS TO INPUT OR OUTPUT

    for (int i=INPUT_RANGE_START; i<=INPUT_RANGE_END; i++) {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 7 as inputs - 5 inputs.
    }

    for (int i=OUTPUT_RANGE_START; i<=OUTPUT_RANGE_END; i++) {
           pinMode(i, OUTPUT);      // define sensor shield pins 8 to 13 as outputs - 5 outputs, plus pin 13 which is the built-in LED.
    }

    randomSeed(analogRead(0));

    for (int pwmOutput = 0; pwmOutput < NUMPWMOUTPUTS; pwmOutput++) {
        flickerDelay[pwmOutput] = random(FLICKER_MIN_TIME, FLICKER_MAX_TIME);
        lastTime[pwmOutput] = 0;
    }
    // Start the serial connection
    Serial.begin(BAUD_RATE); //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
    bus.begin(BAUD_RATE);

    // Initialize PCA9685 board
    pwm.begin();
    pwm.setPWMFreq(PWM_FRAME_RATE);  // This is the maximum PWM frequency
    pwm.writeMicroseconds(0, PWM_NORMAL_LEVEL);

}

void loop(){
    cmri.process();

    // PROCESS SENSORS
    // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required

    // Do not read 0, 1 or 2
    cmri.set_bit(0, !digitalRead(3));  //Bit 0 = address 2001 in JMRI, Light sensor 1
    // cmri.set_bit(1, !digitalRead(4));  //Bit 1 = address 2002 in JMRI, Light sensor 2

    // PROCESS OUTPUTS
    process_outputs();
}
