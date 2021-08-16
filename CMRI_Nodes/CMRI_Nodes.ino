// Include libraries
#include <CMRI.h>
#include <Auto485.h>

// CMRI Settings
#define CMRI_ADDR 1 //CMRI node address in JMRI
#define DE_PIN 2

#define CMRI_INPUTS 24
#define CMRI_OUTPUTS 48

#define BAUD_RATE 19200

// -----------------------------
#define input_range_start 3
#define input_range_end 7
#define output_range_start 8
#define output_range_end 13 // Pin 13 corresponds to the Arduino on-board LED
// -----------------------------

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 64 inputs and 128 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

void setup() {

    // SET PINS TO INPUT OR OUTPUT

    for (int i=input_range_start; i<=input_range_end; i++) {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 7 as inputs - 5 inputs.
    }

    for (int i=output_range_start; i<=output_range_end; i++) {
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
    cmri.set_bit(0, !digitalRead(3));  //Bit 0 = address 1001 in JMRI, IR sensor 1
    cmri.set_bit(1, !digitalRead(4));  //Bit 1 = address 1002 in JMRI, Light Level Sensor 1
    cmri.set_bit(2, !digitalRead(5));  //Bit 2 = address 1003 in JMRI, Home made Light Level Sensor 1

    // PROCESS OUTPUTS
    // Pin 13 corresponds to the Arduino on-board LED
    digitalWrite(13, cmri.get_bit(0));  //Bit 0 = address 1001 in JMRI, LED output 1
}
