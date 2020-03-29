/*

this example shows how to read data from and EBYTE transceiver. 

note you will need to have both transmiter and receiver frequences set to the same channel.

if using and Arduino, you may need 4k7 pullups or inline resistors  ( I dont use this and still works OK, even with 3.3v volts ot 5volts)
## Program Modifications  by @luisgcu March 29 2020

*/


#include "EBYTE.h"
//Primero que nada Seleccionar  la placa a usar remover solo una--> "//". 
//#define ESP32
//#define LEO
#define NANO

#ifdef ESP32                      //If esp32 used     
  #define M0    19                  //GPIO 19
  #define M1    23                  //GPIO 23
  #define AUX   18                  //GPIO 18
  EBYTE Transceiver(&Serial1, M0, M1, AUX);
#endif  

#ifdef LEO                       //If Arduino Leonardo --> TESTED OK
  #define M0  12
  #define M1  11
  #define AUX  10
  EBYTE Transceiver(&Serial1, M0, M1, AUX);
#endif  

#ifdef NANO                     //If Arduino Nano -->  TESTED OK
  #define M0  4
  #define M1  5
  #define AUX  6
  #include <SoftwareSerial.h>
  SoftwareSerial ESerial(2, 3);  
  EBYTE Transceiver(&ESerial, M0, M1, AUX);
#endif

// this is a simple data structure used to send/receive data, edit to your needs
// you can send byte by byte and parse as needed, but that can be problematic
// I highly recommend sending dat via a structure 
struct DATA {
  uint16_t Count =0;
  uint16_t data1=0;
  uint16_t data2=0;
  uint16_t data3=0;

};

int Chan;

// create a data structure
DATA MyData;
void setup() {

// you should set the pin modes, MO, and M1 are to set the usage state of the EBYTE, aux is for reading states of the EBYTE
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);

// wanna see output on the serial monitor? need this line, make sure baud rates here and your serial monitor match
  Serial.begin(115200);

// you MUST begin the serial baud rate for the EBYTE device, you can send data to the EBYTE at faster rates, but you can only program it at 9600
// at least older units could only be programmed at 9600
 delay(400);
#ifdef NANO
  ESerial.begin(9600);
  Serial.println("Starting Receiver Arduino Nano version"); 
#endif

#ifdef ESP32  
  Serial1.begin(9600);
  Serial.println("Starting Receiver ESP32 version");
#endif

#ifdef LEO   
  Serial1.begin(9600);
  Serial.println("Starting Receiver Arduino Leo version");
#endif
// you must initialize the transceiver
// this function will extract all the operating parameters so if changes are made to only 1, all existing parameters are reused
  Transceiver.init();


  // Transceiver.Reset(); // never call this, code not implemented

// unless you are changing the factory settings you need not call any of these functions. Units will work out of the box and let you send and receive data
   Transceiver.SetAirDataRate(ADR_1200); 	// change the air data rate
  // Transceiver.SetAddressH(0);		// set the high address byte
   Transceiver.SetAddressL(4);		// set the low address byte
   Transceiver.SetChannel(15);			// set the channel (0-32 is pretty typical)
   Transceiver.SaveParameters(PERMANENT);	// save the parameters to the EBYTE EEPROM, you can save temp if periodic changes are needed

// if you want to see the units settings, otherwise never need to call this
   Transceiver.PrintParameters();  
#ifdef NANO
  ESerial.flush();     //this is important just to fluhs anything on the serial1 port
#endif
#ifdef ESP32  
  Serial1.flush();     //this is important just to fluhs anything on the serial1 port
  
#ifdef  LEO   
  Serial1.flush();     //this is important just to fluhs anything on the serial1 port
#endif  
  
#endif  
  delay(100); 
}

void loop() {

// for reading data, it's good to see if the serail object has any data
  if (ESerial.available()) {

// one simple call to get the data structure, note the mandatory use of the & for reference passing
    Transceiver.GetStruct(&MyData, sizeof(MyData));

// now process the data as needed
    Serial.print("Count: "); Serial.print(MyData.Count); Serial.print(", ");
    Serial.print("data1: "); Serial.print(MyData.data1); Serial.print(", ");
    Serial.print("data2: "); Serial.print(MyData.data2);  Serial.print(", ");
    Serial.print("data3: "); Serial.println(MyData.data3); 
    Serial.println();
  }/*
  else {
    delay(1000);
    Serial.println("Searching: ");
  } */
}
