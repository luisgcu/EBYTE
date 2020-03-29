/*

this example shows how to send data to an EBYTE transceiver. 

note you will need to have both transmiter and receiver frequences set to the same channel.

if using and Arduino, you may need 4k7 pullups or inline resistors

*/

// include software serial libraries for Arduino
// note the Teensy uses a #define to create the serial object
#include <SoftwareSerial.h>
#include "EBYTE.h"

int Chan;

// this is a simple data structure used to send/receive data, edit to your needs
// you can send byte by byte and parse as needed, but that can be problematic
// I highly recommend sending dat via a structure

struct DATA {
  unsigned long Count;
  int Bits;
  float Volts;
  float Amps;

};

// create a data structure
DATA MyData;

// create a serial object for Arduino, again Teensy uses a much simpler #define scenario
//SoftwareSerial ESerial(2, 3);

// Now you MUST create the transceiver object and you must pass in the serail object use the & to pass by reference
// usage for teensy is the exact same
// M0, M1, and Aux pins are next
//EBYTE Transceiver(&ESerial, 4, 5, 6);
EBYTE Transceiver(&Serial1, 12, 11, 10);

void setup() {

  // you should set the pin modes, MO, and M1 are to set the usage state of the EBYTE, aux is for reading states of the EBYTE
  //pinMode(4, OUTPUT);
  //pinMode(5, OUTPUT);
  //pinMode(6, INPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, INPUT);
  
// wanna see output on the serial monitor? need this line, make sure baud rates here and your serial monitor match
  Serial.begin(115200);
  //while (!Serial) ; // Wait for serial port to be available
  delay(400);
// you MUST begin the serial baud rate for the EBYTE device, you can send data to the EBYTE at faster rates, but you can only program it at 9600
// at least older units could only be programmed at 9600
  //ESerial.begin(9600);
  Serial1.begin(9600);

  Serial.println("Starting Sender");

  
// you must initialize the transceiver
// this function will extract all the operating parameters so if changes are made to only 1, all existing parameters are reused
  if (!Transceiver.init())
 Serial.println("init failed"); 
  
  // unless you are changing the factory settings you need not call any of these functions. Units will work out of the box and let you send and receive data
  // Transceiver.SetAirDataRate(ADR_8K); 	// change the air data rate
  // Transceiver.SetAddressH(0);		// set the high address byte
  // Transceiver.SetAddressL(0);		// set the low address byte
  // Transceiver.SetChannel(2);			// set the channel (0-32 is pretty typical)
  // Transceiver.SaveParameters(PERMANENT);	// save the parameters to the EBYTE EEPROM, you can save temp if periodic changes are needed

   // if you want to see the units settings, otherwise never need to call this
   Transceiver.PrintParameters();

}

void loop() {

  // build your data structure with your data
  MyData.Count++;
  MyData.Bits = analogRead(A0);
  MyData.Volts = MyData.Bits * ( 5.0 / 1024.0 );

  // one simple call to send the data structure, note the mandatory use of the & for reference passing
  Transceiver.SendStruct(&MyData, sizeof(MyData));
  // it might be nice to see when the data is sent
  Serial.print("Sending: "); Serial.println(MyData.Count);
  delay(15000);
  
}
