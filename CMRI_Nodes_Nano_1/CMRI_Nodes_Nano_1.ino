// Include libraries
#include <CMRI.h>
#include <Auto485.h>

// CMRI Settings
#define CMRI_ADDR 2 //CMRI node address in JMRI
#define DE_PIN 2

#define CMRI_INPUTS 24
#define CMRI_OUTPUTS 48

#define BAUD_RATE 19200

#define NUMOUTPUTS  1

// -----------------------------
#define INPUT_RANGE_START    0
#define INPUT_RANGE_END      6
#define OUTPUT_RANGE_START   7
#define OUTPUT_RANGE_END    13 // Pin 13 corresponds to the Arduino on-board LED
// -----------------------------

bool lastState[NUMOUTPUTS] = {LOW};
bool requiredState[NUMOUTPUTS] = {LOW};

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 24 inputs and 48 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

void setup() {

    // SET PINS TO INPUT OR OUTPUT

    for (int i=INPUT_RANGE_START; i<=INPUT_RANGE_END; i++) {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 7 as inputs - 5 inputs.
    }

    for (int i=OUTPUT_RANGE_START; i<=OUTPUT_RANGE_END; i++) {
           pinMode(i, OUTPUT);      // define sensor shield pins 8 to 13 as outputs - 5 outputs, plus pin 13 which is the built-in LED.
    }

    // Start the serial connection
    Serial.begin(BAUD_RATE); //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
    bus.begin(BAUD_RATE);
}

void loop(){
    cmri.process();

    // PROCESS SENSORS
    // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required

    // Do not read 0, 1 or 2
     cmri.set_bit(0, !digitalRead(3));  //Bit 0 = address 2001 in JMRI, Light sensor 1
    // cmri.set_bit(1, !digitalRead(4));  //Bit 1 = address 2002 in JMRI, Light sensor 2

    // PROCESS OUTPUTS
    requiredState[0] = cmri.get_bit(0);
    if (lastState[0] != requiredState[0]) {
        // Pin 13 corresponds to the Arduino on-board LED
        digitalWrite(13, requiredState[0]);  //Bit 0 = address 2001 in JMRI, LED output 1
        lastState[0] = requiredState[0];
    }
}
