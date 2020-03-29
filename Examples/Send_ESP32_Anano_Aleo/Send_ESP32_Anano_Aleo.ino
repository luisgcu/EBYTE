/*

this example shows how to send data to an EBYTE transceiver. 

note you will need to have both transmiter and receiver frequences set to the same channel.

if using and Arduino, you may need 4k7 pullups or inline resistors  ( I dont use this and still works OK, even with 3.3v volts ot 5volts)
## Program editions by @luisgcu March 29 2020
*/


#include "EBYTE.h"
//#include <HardwareSerial.h>

//Primero que nada Seleccionar  la placa a usar remover solo una--> "//". 

//#define ESP32                     // ESP 32      TESTED OK
#define LEO                       // Arduino Leonardo
//#define NANO                        // Arduino Nano

 //#define RX1   16                // En esp32 hay que configurar hardwareSerial para serial1 sea GPIO 16 y 17
 //#define TX1   17
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


int Chan;

                                // this is a simple data structure used to send/receive data, edit to your needs
                                // you can send byte by byte and parse as needed, but that can be problematic
                                // I highly recommend sending dat via a structure

struct DATA {
  unsigned  int Count=0;        //Make sure to properly pick the correct data type for your extruture
  unsigned  int data1=0;        //Also  be aware data lengh change from one platform to another 
  unsigned  int data2=0;        //For instance "unsigned  int" is 2 bytes in arduino uno/nano, but ESP32 is 4 bytes
  unsigned  int data3=0;        //If we want to correctly receive this structure in ESP32 then we need to declare variables as "uint16_t"
                                // Mas informacion sobre los data types aqui: https://github.com/espressif/arduino-esp32/files/2276662/DataTypes.pdf
};
// create a data structure
DATA MyData;
void setup() {
                                // you should set the pin modes, MO, and M1 are to set the usage state of the EBYTE, aux is for reading states of the EBYTE  
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);  
                                 // wanna see output on the serial monitor? need this line, make sure baud rates here and your serial monitor match
  Serial.begin(115200);
  //while (!Serial) ;           // Wait for serial port to be available
  delay(400);
#ifdef NANO
  ESerial.begin(9600);
  Serial.println("Starting Sender Arduino Nano version"); 
#endif

#ifdef ESP32  
  Serial1.begin(9600);
  Serial.println("Starting Sender ESP32 version");
#endif

#ifdef LEO   
  Serial1.begin(9600);
  Serial.println("Starting Sender Arduino Leo version");
#endif
  

  if (!Transceiver.init())
 Serial.println("init failed");   
  
  // unless you are changing the factory settings you need not call any of these functions. Units will work out of the box and let you send and receive data
   Transceiver.SetAirDataRate(ADR_1200); 	// change the air data rate  ( debe ser igual en el modulo TX y RX
  // Transceiver.SetAddressH(0);		// set the high address byte
   Transceiver.SetAddressL(4);		// set the low address byte          ( La direccion 4 es broadcast)
   Transceiver.SetChannel(15);			// set the channel (0-32 is pretty typical)  ( Canal de frecuencia)
   Transceiver.SaveParameters(PERMANENT);	// save the parameters to the EBYTE EEPROM, you can save temp if periodic changes are needed
   // if you want to see the units settings, otherwise never need to call this
   Transceiver.PrintParameters();          //( imprimer los parametros que estan en la memoria del modulo)
   
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
  // build your data structure with your data
  MyData.Count++;
  MyData.data1 =123;
  MyData.data2 =432;
  MyData.data3 =567;
  //MyData.data1 = analogRead(A0);
  //MyData.data2 = MyData.data1 * ( 5.0 / 1024.0 );

  // one simple call to send the data structure, note the mandatory use of the & for reference passing
  Transceiver.SendStruct(&MyData, sizeof(MyData));
  // it might be nice to see when the data is sent
  Serial.print("Sending: "); Serial.print(MyData.Count); Serial.print(" , ");
  Serial.print(MyData.data1); Serial.print(" , "); Serial.println(MyData.data2);
  delay(15000);
  
}
