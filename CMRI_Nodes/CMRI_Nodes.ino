// Include libraries
#include <CMRI.h>
#include <Auto485.h>
#include <TimerOne.h>

// CMRI Settings
#define CMRI_ADDR 1 //CMRI node address in JMRI
#define DE_PIN 2

#define CMRI_INPUTS 24
#define CMRI_OUTPUTS 48

#define BAUD_RATE 115200
#define SERIAL_BAUD_RATE 19200

// -----------------------------
#define input_range_start 3
#define input_range_end 7
#define output_range_start 8
#define output_range_end 13 // Pin 13 corresponds to the Arduino on-board LED
// -----------------------------

// -----------------------------
// LED states
// -----------------------------
#define OFF         0 // Light switched off.
#define ON          1 // Light switched on.

// -----------------------------
// Interrupt Period
// -----------------------------

#define INT_PERIOD          5000    // Number of micro seconds, so 5000 is once every 5 milli seconds (200 times a second), so the interrupt will activate 200 times a second.
                                    // The Arduino has a receive buffer of 64 characters, hence with 200 interrupts a second it can receive 12800 characters, or approx. 128000 bits.
                                    // i.e. more than 115200 baud would be capable of.

// -----------------------------
// Global variables
// -----------------------------
volatile bool IR_sensor = OFF;
volatile bool light_sensor_1 = OFF;
volatile bool light_sensor_2 = OFF;
volatile bool arduino_LED = OFF;
bool arduino_last_state = OFF;

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 64 inputs and 128 outputs
CMRI cmri(CMRI_ADDR, CMRI_INPUTS, CMRI_OUTPUTS, bus);

// ----------------------------------------------
// ------------- FUNCTION PROTOTYPES ------------
// ----------------------------------------------

void setup(void);
void loop(void);
void send_and_receive_CMRI(void);
// void readSensors(void);
// void sendToCMRI(void);
// void readFromCMRI(void);
void processOutputs(void);

// ----------------------------------------------
// ----------------- FUNCTIONS ------------------
// ----------------------------------------------

void setup(void) {

    // SET PINS TO INPUT OR OUTPUT

    for (int i=input_range_start; i<=input_range_end; i++) {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 7 as inputs - 5 inputs.
    }

    for (int i=output_range_start; i<=output_range_end; i++) {
           pinMode(i, OUTPUT);      // define sensor shield pins 8 to 13 as outputs - 5 outputs, plus pin 13 which is the built-in LED.
    }

    // Start the serial connections
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.begin(SERIAL_BAUD_RATE); //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
#endif
    bus.begin(BAUD_RATE);

    // Initialise the timer interrupt
    Timer1.initialize(INT_PERIOD);
    Timer1.attachInterrupt(send_and_receive_CMRI);

}

void loop(void) {
    // readSensors();
    processOutputs();
}

void send_and_receive_CMRI(void) {
    cmri.process();
    // Read sensors
    // Do not read 0, 1 or 2
    IR_sensor = digitalRead(3);         // Pin 3 on Arduino
    light_sensor_1 = digitalRead(4);    // Pin 4 on Arduino
    light_sensor_2 = digitalRead(5);    // Pin 5 on Arduino
    // Send to CMRI
    cmri.set_bit(0, !IR_sensor);        //Bit 0 = address 1001 in JMRI, IR sensor 1
    cmri.set_bit(1, !light_sensor_1);   //Bit 1 = address 1002 in JMRI, Light Level Sensor 2
    cmri.set_bit(2, !light_sensor_2);   //Bit 2 = address 1003 in JMRI, Home made Light Level Sensor 3
    // sendToCMRI();
    //Read from CMRI
    // readFromCMRI();
    arduino_LED = cmri.get_bit(0);      // Bit 0 = address 1001 in JRMI, LED output 1
}

// void readSensors(void) {
    // Do not read 0, 1 or 2
    // IR_sensor = digitalRead(3);         // Pin 3 on Arduino
    // light_sensor_1 = digitalRead(4);    // Pin 4 on Arduino
    // light_sensor_2 = digitalRead(5);    // Pin 5 on Arduino
// }

// void sendToCMRI(void) {
    // cmri.set_bit(0, !IR_sensor);        //Bit 0 = address 1001 in JMRI, IR sensor 1
    // cmri.set_bit(1, !light_sensor_1);   //Bit 1 = address 1002 in JMRI, Light Level Sensor 2
    // cmri.set_bit(2, !light_sensor_2);   //Bit 2 = address 1003 in JMRI, Home made Light Level Sensor 3
// }

// void readFromCMRI(void) {
    // arduino_LED = cmri.get_bit(0);      // Bit 0 = address 1001 in JRMI, LED output 1
// }

void processOutputs(void) {
    if (arduino_last_state != arduino_LED) {
        arduino_last_state = arduino_LED;
        digitalWrite(13, arduino_LED);  // Pin 13 corresponds to the Arduino on-board LED
    }
}
