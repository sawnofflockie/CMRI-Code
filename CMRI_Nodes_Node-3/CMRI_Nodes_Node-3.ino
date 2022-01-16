
// Include libraries
#include <CMRI.h>
#include <Auto485.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>

//#define ENABLE_DEBUG_OUTPUT

// CMRI Settings
#define CMRI_ADDR              3 //CMRI node address in JMRI
#define DE_PIN                 2

#define CMRI_INPUTS           24
#define CMRI_OUTPUTS          48

#define BAUD_RATE          57600
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
// Sound Output Pins
// -----------------------------

#define LOUD_PIN               4
#define SOFT_PIN               5

// -----------------------------
// Interrupt Period
// -----------------------------

#define INT_PERIOD          5000    // Number of micro seconds, so 5000 is once every 5 milli seconds (200 times a second), so the interrupt will activate 200 times a second.
                                    // The Arduino has a receive buffer of 64 characters, hence with 200 interrupts a second it can receive 12800 characters, or approx. 128000 bits.
                                    // i.e. more than 115200 baud would be capable of.

// -----------------------------
// Declare global variables
// -----------------------------

volatile bool lightLevel1 = OFF;
volatile bool lightLevel2 = OFF;
volatile bool soundEnable = OFF;

int lastTimeOfDay = DAYTIME;

int min;    // Smallest case
int max;    // Largest case

unsigned long currentTime;  // The time, in milliseconds, of the current processing loop.
unsigned long waitStart;    // The time of the start id a wait period, in milliseconds.
unsigned long waitPeriod;   // How long to wait, in milliseconds.
unsigned long endTime;      // Time at which the wait period ends, in milliseconds.

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
void tone(int mark, int space, int cycles, int port);
void upTone(int start, int end, int increment, int port);
void downTone(int start, int end, int increment, int port);
void noise(int tone, int dly, int cycles, int port);
void cricket_1(void);
void cricket_2(void);
void bird_1(void);
void bird_2(void);
void bird_3(void);
void bird_4(void);
void bird_5(void);
void bird_6(void);
void chooseSound(void);
bool soundOff(void);
void startWait(int timeToWait);
void setMinMax(int timeOfDay);

// ----------------------------------------------
// ----------------- FUNCTIONS ------------------
// ----------------------------------------------

void setup(void) {

    randomSeed(analogRead(0)); // Reads noise on an unconnected pin to seed the random number generator.

    // Start the serial connections
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.begin(SERIAL_BAUD_RATE); // Baud rate of the serial monitor used for debug output.
#endif
    // bus.begin(BAUD_RATE, SERIAL_8N2); // May need this version (8 bit, no parity, 2 stop bits) if JMRI doesn't understand the standard single stop bit.
    bus.begin(BAUD_RATE); // Ensure this matches the baud rate in JMRI.

    // Initialise the timer interrupt
    Timer1.initialize(INT_PERIOD);
    Timer1.attachInterrupt(readFromCMRI);

    // Initialize PCA9685 board
    pwm.begin();
    pwm.setPWMFreq(PWM_FRAME_RATE);  // This is the maximum PWM frequency
    
    // Set up sound output pins
    pinMode(LOUD_PIN, OUTPUT); // Loud - Pin D2
    pinMode(SOFT_PIN, OUTPUT); // Soft - Pin D3
}

void loop(void){
    int timeOfDay;

    timeOfDay = setDayTime(lastTimeOfDay);
    lastTimeOfDay = illuminateLED(timeOfDay, lastTimeOfDay);
    setMinMax(timeOfDay);
    chooseSound();
#ifdef ENABLE_DEBUG_OUTPUT
//    Serial.print("Time of day = ");
//    Serial.println(timeOfDay);
//    Serial.print("min = ");
//    Serial.print(min);
//    Serial.print(", max = ");
//    Serial.println(max);
    Serial.print("lightLevel1 = ");
    Serial.print(lightLevel1);
    Serial.print(", lightLevel2 = ");
    Serial.println(lightLevel2);
#endif
}

void readFromCMRI(void) {
    // Called via timer1 interrupt.
    cmri.process();
    lightLevel1 = cmri.get_bit(0);  // Bit 0 = address 3001 in JRMI
    lightLevel2 = cmri.get_bit(1);  // Bit 1 = address 3002 in JMRI
    soundEnable = cmri.get_bit(2);  // Bit 2 = address 3003 in JMRI
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
            } else {
                level = OFF;
            }
            pwm.writeMicroseconds(pwmOutput, level);
        }
        lastTimeOfDay = timeOfDay;
    }
    return lastTimeOfDay;
}

void tone(int mark, int space, int cycles, int port) {
    for(int i=0; i<cycles; i++) {
        digitalWrite(port, HIGH);
        delayMicroseconds(mark);
        digitalWrite(port, LOW);
        delayMicroseconds(space);
    }
}

void upTone(int start, int end, int increment, int port) {
    for(int i=start; i>end; i-=increment) {
        digitalWrite(port, HIGH);
        delayMicroseconds(start - i);
        digitalWrite(port, LOW);
        delayMicroseconds(start - i);
    }
}

void downTone(int start, int end, int increment, int port) {
    for(int i = start; i < end; i+=increment) {
        digitalWrite(port, HIGH);
        delayMicroseconds(start + i);
        digitalWrite(port, LOW);
        delayMicroseconds(start + i);
    }
}

void noise(int tone, int dly, int cycles, int port) {
    for(int i=0; i<cycles; i++) {
        delayMicroseconds(random(1,dly));
        digitalWrite(port, HIGH);
    }
}

void cricket_1(void) {
    for (int j=0; j < random(3, 6); j++) {
        for (int i=0; i <10; i++) {
            tone(86, 96, 62, SOFT_PIN);
            tone(72, 132, 62, SOFT_PIN);
        }
        delay(100);
    }
    startWait(random(500, 1000));
}

void cricket_2(void) {
    for (int i=0; i <30; i++) {
        tone(86, 96, 62, SOFT_PIN);
        tone(72, 132, 62, SOFT_PIN);
    }
    startWait(random(500, 1000));
}

void bird_1(void) {
    for (int i=0; i <2; i++) {
        tone(171, 181, 750, LOUD_PIN);
        delay(10);
        tone(185, 195, 750, LOUD_PIN);
        delay(300);
    }
    startWait(random(100, 500));
}

void bird_2(void) {
    for (int i=0; i < 10; i++) {
        upTone(random(300, 400), 200, 1, LOUD_PIN);
        delay(2);
    }
    delay(90);
    for (int i=0; i < 5; i++) {
        downTone(200, random(300, 400), 1, LOUD_PIN);
        delay(2);
    }
    delay(90);
    for (int i=0; i < 10; i++) {
        upTone(random(300, 400), 200, 1, LOUD_PIN);
        delay(2);
    }
    startWait(random(100, 500));
}

void bird_3(void) {
    for (int i=0; i < 10; i++) {
        upTone(random(270, 370), 150, 1, LOUD_PIN);
        delay(random(60, 120));
    }
}

void bird_4(void) {
    for (int i=0; i < 5; i++) {
        downTone(150, random(220, 320), 1, LOUD_PIN);
        delay(random(60, 120));
    }
}

void bird_5(void) {
    for (int i=0; i < random(5, 10); i++) {
        downTone(50, random(120, 220), 1, LOUD_PIN);
        delay(random(60, 120));
    }
}

void bird_6(void) {
    upTone(2000, 200, 200, LOUD_PIN);
    noise(500, 450, 300, LOUD_PIN);
    noise(500, 30, 2500, LOUD_PIN);
    noise(500, 350, 300, LOUD_PIN);
    downTone(1000, 2000, 50, LOUD_PIN);
    startWait(1000);
}

void chooseSound(void) {
    if (!soundOff()) {
        switch(random(min, max)) {
            case 0:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 0: ");
                  Serial.println("bird 6");
                #endif
                bird_6();
            break;
            case 2:
            case 3:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 2 & 3: ");
                  Serial.println("cricket 1");
                #endif
                cricket_1();
            break;
            case 4:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 4: ");
                  Serial.println("cricket 2");
                #endif
                cricket_2();
            break;
            case 5:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.println("case 5");
                #endif
                startWait(5000);
            break;
            case 7:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 7: ");
                  Serial.println("cricket 2");
                #endif
                cricket_2();
            break;
            case 8:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 8: ");
                  Serial.println("bird 2");
                #endif
                bird_2();
            break;
            case 9:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 9: ");
                  Serial.println("bird 4");
                #endif
                bird_4();
            break;
            case 10:
            case 11:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 10 & 11: ");
                  Serial.println("bird 1");
                #endif
                bird_1();
            break;
            case 12:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 12: ");
                  Serial.println("bird 4");
                #endif
                bird_4();
            break;
            case 13:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 12 & 13: ");
                  Serial.println("bird 2");
                #endif
                bird_2();
            break;
            case 14:
            case 15:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 14 & 15: ");
                  Serial.println("bird 3");
                #endif
                bird_3();
            break;
            case 16:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 16: ");
                  Serial.println("bird 5");
                #endif
                bird_5();
            break;
            case 17:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.print("case 17: ");
                  Serial.println("bird 3");
                #endif
                bird_3();
            break;
            default:
                #ifdef ENABLE_DEBUG_OUTPUT
                  Serial.println("default");
                #endif
                startWait(100 + random(500, 2000));
            break;
        }
    }
}

bool soundOff(void) {
    currentTime = millis();
    if ((currentTime > waitStart + waitPeriod) && soundEnable) {
        return false;
    } else return true;
}

void startWait(int timeToWait) {
    waitStart = millis();
    waitPeriod = (unsigned long) timeToWait;
}

void setMinMax(int timeOfDay) {
    switch (timeOfDay) {
        case DAYTIME:
            min = 2;
            max = 20;
        break;
        case EVENING:
            min = 1;
            max = 9;
        break;
        case NIGHT:
            min = 0;
            max = 8;
        break;
        case DAWN:
            min = 6;
            max = 13;
        break;
    }
}
