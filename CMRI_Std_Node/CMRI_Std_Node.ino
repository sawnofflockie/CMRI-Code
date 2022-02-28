// ========================================
// Standard CMRI sensor node (Arduino Nano)
// ========================================

// Include libraries
#include <Wire.h>
#include <CMRI.h>
#include <Auto485.h>

// CMRI Settings
#define CMRI_ADDR 1 //CMRI node address in JMRI
#define DE_PIN 2

#define CMRI_INPUTS 24
#define CMRI_OUTPUTS 48

#define BAUD_RATE 19200
#define SERIAL_BAUD_RATE 19200

// -----------------------------
#define INPUT_RANGE_START 3
#define INPUT_RANGE_END 12
// -----------------------------

// -----------------------------
// States
// -----------------------------
#define OFF         0
#define ON          1

// -----------------------------
// Global variables
// -----------------------------
bool sensor_1 = OFF;
bool sensor_2 = OFF;
bool sensor_3 = OFF;
bool sensor_4 = OFF;
bool sensor_5 = OFF;
bool sensor_6 = OFF;
bool sensor_7 = OFF;
bool sensor_8 = OFF;
bool sensor_9 = OFF;
bool sensor_10 = OFF;
int light_state = OFF;

// -----------------------------
// Function prototypes
// -----------------------------
void setup(void);
void loop(void);
void readDigitalSensors(void);
void sendToCMRI(void);

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 64 inputs and 128 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

void setup(void) {

    // SET PINS TO INPUT OR OUTPUT

    for (int i=INPUT_RANGE_START; i<=INPUT_RANGE_END; i++) {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 12 as inputs - 10 inputs.
    }
    pinMode(13, OUTPUT);

    // Start the serial connection
    Serial.begin(SERIAL_BAUD_RATE);     // Used for debug output.
    // bus.begin(BAUD_RATE, SERIAL_8N2); // May need this version (8 bit, no parity, 2 stop bits) if JMRI doesn't understand the standard single stop bit.
    bus.begin(BAUD_RATE);               // Ensure this matches the baud rate in JMRI.
}

void loop(void){
    cmri.process();
    sendToCMRI();
    readDigitalSensors();
}

void readDigitalSensors(void) {
    digitalWrite(13, light_state);
    // Do not read 0, 1 or 2
    // sensor_1 = digitalRead(3);      // Pin 3 on Arduino
    // sensor_2 = digitalRead(4);      // Pin 4 on Arduino
    // sensro_3 = digitalRead(5);      // Pin 5 on Arduino
    // sensor_4 = digitalRead(6);      // Pin 6 on Arduino
    // sensor_5 = digitalRead(7);      // Pin 7 on Arduino
    // sensor_6 = digitalRead(8);      // Pin 8 on Arduino
    // sensor_7 = digitalRead(9);      // Pin 9 on Arduino
    // sensor_8 = digitalRead(10);     // Pin 10 on Arduino
    // sensor_9 = digitalRead(11);     // Pin 11 on Arduino
    // sensor_10 = digitalRead(12);    // Pin 12 on Arduino
}

void sendToCMRI(void) {
    light_state = cmri.get_bit(0);
    cmri.set_bit(0, !light_state);     //Bit 0 = address 1001 in JMRI/CMRI
    // cmri.set_bit(1, !sensor_2);     //Bit 1 = address 1002 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_3);     //Bit 2 = address 1003 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_4);     //Bit 2 = address 1004 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_5);     //Bit 2 = address 1005 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_6);     //Bit 2 = address 1006 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_7);     //Bit 2 = address 1007 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_8);     //Bit 2 = address 1008 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_9);     //Bit 2 = address 1009 in JMRI/CMRI
    // cmri.set_bit(2, !sensor_10);    //Bit 2 = address 1010 in JMRI/CMRI
}
