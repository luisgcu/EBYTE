/*

This is Simple GPS SENDER USING EBYTE LIB.
THE SAMPLE PROGRAM DO NO HAVE ACK OPTION TO CONFIRM THE DATA HAVE BEEN RECEIVED.
THIS TEST WAS USED WITH ARDUINO LEONARDO 
GPS CONNECTED AT PIN 8(RX-LEON) 
## Program editions by @luisgcu March 29 2020
*/


#include "EBYTE.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 9);              // RX, TX  //this is gps 

//Primero que nada Seleccionar  la placa a usar remover solo una--> "//". 

//#define ESP32                             // ESP 32      TESTED OK
#define LEO                                 // Arduino Leonardo
//#define NANO                              // Arduino Nano

 //#define RX1   16                         // En esp32 hay que configurar hardwareSerial para serial1 sea GPIO 16 y 17
 //#define TX1   17
#ifdef ESP32                                //If esp32 used     
  #define M0    19                          //GPIO 19
  #define M1    23                          //GPIO 23
  #define AUX   18                          //GPIO 18
  EBYTE Transceiver(&Serial1, M0, M1, AUX);
#endif  

#ifdef LEO                                  //If Arduino Leonardo --> TESTED OK
  #define M0  12
  #define M1  11
  #define AUX  10
  EBYTE Transceiver(&Serial1, M0, M1, AUX);
#endif  

#ifdef NANO                                 //If Arduino Nano -->  TESTED OK
  #define M0  4
  #define M1  5
  #define AUX  6
  #include <SoftwareSerial.h>
  SoftwareSerial ESerial(2, 3);  
  EBYTE Transceiver(&ESerial, M0, M1, AUX);
#endif  

#define red 2
#define blue 3
#define green 4

TinyGPSPlus gps;
int         Chan;
const int   SendInterval          =  5; //Check every 150 seconds if position had changed
int         SendIntervalCount     = 0;
int         ledState              = HIGH;
int         replicar1             = 0;

double      oldlat1 ;
double      oldlong1;                              

struct      Gpsdata {                  //Structure to hold latidude and longitude to be sent using E32/E44 modules
  float     latitude;
  float     longitude; 
};

Gpsdata MyGpsdata;                    // create a data structure

bool        InputTrigger;             // not used
uint8_t*GpsPayload;

unsigned long previousMillis = 0;    // will store last time LED was updated
void setup() {
                                     // you should set the pin modes, MO, and M1 are to set the usage state of the EBYTE, aux is for reading states of the EBYTE  
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);                                  
  Serial.begin(115200); 
  mySerial.begin(9600);             // init GPS serial port (softwareSerial)
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
  // Transceiver.SetAddressH(0);		      // set the high address byte
   Transceiver.SetAddressL(4);		        // set the low address byte          ( La direccion 4 es broadcast)
   Transceiver.SetChannel(15);			      // set the channel (0-32 is pretty typical)  ( Canal de frecuencia)
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
  // Led RGB init 
  pinMode(red,OUTPUT);
  pinMode(blue,OUTPUT);
  pinMode(green,OUTPUT);
  digitalWrite(red,HIGH);
  digitalWrite(blue,HIGH);
  digitalWrite(green,HIGH);
  //Test Led
  Blink(blue, 30, 3); //blink LED 3 times, 40ms between blinks
  Blink(red, 20, 3); //blink LED 3 times, 40ms between blinks
  Blink(green, 20, 4); //blink LED 3 times, 40ms between blinks
}

void loop() {
  while (mySerial.available() > 0)      //If GPS data?
    if (gps.encode(mySerial.read()))
    {
      SendIntervalCount++;
      Serial.println("Tick");
       if (SendIntervalCount > SendInterval)
       {
        // Serial.print("SendIntervalCount:=");   Serial.println (SendIntervalCount);
        SendIntervalCount = 0;
        Serial.println("Tick..Tick..");
        if (gps.location.isValid())
          // if (gps.location.isUpdated() && gps.location.isValid() ) 
          {
            double lat1 = gps.location.lat();
            double lon1 = gps.location.lng();
            double distance = gps.distanceBetween(lat1, lon1, oldlat1, oldlong1);
            // Serial.print("Distance:  ");
            // Serial.println(distance,2);
            // Serial.println("enviando.....");
            Gps_sending();                       //Call the subrutine to send data
        if  (distance >= 30){
            oldlat1 = lat1;
            oldlong1 = lon1;
            Serial.println("enviando...");
            //Gps_sending();                  //as alternative if we want send data when distance changed
           } 
           else 
           {
             Serial.println("Distance Delta too small");
           }         
        }
        else
        {
          //no gps fix blue led blink
          Serial.println(F("Location not valid"));
          Blink(red, 20, 4); //blink LED 3 times, 40ms between blinks

        }
       // Serial1.flush();

      }
    }
  
}


void Gps_sending() {
  MyGpsdata.latitude  = gps.location.lat();
  MyGpsdata.longitude = gps.location.lng();  
  Transceiver.SendStruct(&MyGpsdata, sizeof(MyGpsdata));
  // it might be nice to see when the data is sent
  Serial.print("Sending: "); Serial.print(MyGpsdata.latitude); Serial.print(" , ");
  Serial.println(MyGpsdata.longitude);    
  Blink(blue, 20, 4); //blink LED 3 times, 40ms between blinks     
}


void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i = 0; i < loops; i++)
  {
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
  }
}
