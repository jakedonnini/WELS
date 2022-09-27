

//This version of the WELS V2 recever code has the vibrators pulse instead of dim
//currently this doesn't have direction fuctionality.


#include <MPU6050.h>
#include "I2Cdev.h"
#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
#include "MPU9250.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//--- mag sense
MPU9250 IMU(Wire,0x68);
int status; 
float magPitch, magRoll;
float magX_hor, magY_hor;
float magX_damp, magY_damp;
int magHeading;
int Heading;


//--- radio
RF24 radio(7,8); // CE, CSN for mega
RF24Network network(radio);      // Include the radio in the network
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t rightHand01 = 01;      // Address of the other node in Octal format
const uint16_t leftHand012 = 02;
const uint16_t rightFoot04 = 03;      // 0X for child, 0XX for grand child, use only child 
const uint16_t leftFoot022 = 04;

//--- vib setup
unsigned long timeAftRH = 0;
unsigned long timeAftLH = 0;
unsigned long timeAftRF = 0;
unsigned long timeAftLF = 0;
int vpRH[] = {2, 3, 4}; //Pwm pins
int vpLH[] = {5, 6, 9}; 
int vpRF[] = {11, 10, 13};
int vpLF[] = {44, 45, 46};
int pinCount = 3;
int vibAngleRH, vibAngleLH, vibAngleRF, vibAngleLF;
float vibAngleRawRH, vibAngleRawLH, vibAngleRawRF, vibAngleRawLF;
int vibStateRH = LOW;
int vibStateLH = LOW;
int vibStateRF = LOW;
int vibStateLF = LOW;
int pos1, pos2, posState;
bool vibePulse = false;

// datapack unload
struct DataPack {
          int distance;
          int magValIn;
    };

void setup() {
  
  Serial.begin(9600);
  SPI.begin();
  //--- radio setup
  radio.begin();
  network.begin(90, master);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  pinMode(8, OUTPUT);

  //--- for loops to inizlise each vib pin
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpRH[thisPin], OUTPUT);
  }
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpLH[thisPin], OUTPUT);
  }
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpRF[thisPin], OUTPUT);
  }
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpLF[thisPin], OUTPUT);
  }

  //---mag sanity check
  while(!Serial) {}
  
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}


void loop() {
  network.update(); //update network
  

    
  DataPack dataPack; // declare data pack
  RF24NetworkHeader header;
  RF24NetworkHeader ping(rightHand01);

  struct DataOut{
    bool message = true;
    int magValOut = 0;
  };
  DataOut dataOut;
  
  
  //--- make radio connection
  if (network.available()) { 
   while (network.available()) { // if radio is found

      network.read(header, &dataPack, sizeof(dataPack));   //read radio


    
    //--- vibration calc
   
     
    
     
    
    //------------------Right Hand----------------------
    
    if (header.from_node == 01) { //rh
    
    //network.write(ping, &dataOut, sizeof(dataOut)); //send to right hand
      if (vibePulse == true) {
       unsigned long timeElapsedRH = millis();
      if (dataPack.distance < 100) { // distance limit, increas to increase limit
            if (timeElapsedRH - timeAftRH >= dataPack.distance*5){  //create pulsing 
              timeAftRH = timeElapsedRH;
                   if (vibStateRH == LOW) {
                vibStateRH = HIGH;
              } else {
                vibStateRH = LOW;
              } // to else
             } // to if time of vib
         digitalWrite(vpRH[0], vibStateRH);  // output vib 
      } //if range limit
        else  { digitalWrite(vpRH[0], LOW); //make sure off
                digitalWrite(vpRH[1], LOW);
                digitalWrite(vpRH[2], LOW);
      }  // to else

      } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpRH[0], ((100 - dataPack.distance) * 2.55)); 
            }  else  {
              digitalWrite(vpRH[0], LOW); //make sure off
            }  // to else
          }
    
    }

    
  //------------------Left Hand----------------------
     if (header.from_node == 02) {

      if (vibePulse == true) {
      unsigned long timeElapsedLH = millis();
      if (dataPack.distance < 100) { // distance limit, increas to increase limit
            if (timeElapsedLH - timeAftLH >= dataPack.distance*5){  //create pulsing 
              timeAftLH = timeElapsedLH;
                   if (vibStateLH == LOW) {
                vibStateLH = HIGH;
              } else {
                vibStateLH = LOW;
              } // to else
             } // to if time of vib
         digitalWrite(vpLH[0], vibStateLH);  // output vib 
      } //if range limit
        else  { digitalWrite(vpLH[0], LOW); //make sure off
                digitalWrite(vpLH[1], LOW);
                digitalWrite(vpLH[2], LOW);
      }  // to else

      } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpLH[0], ((100 - dataPack.distance) * 2.55)); 
            }  else  {
              digitalWrite(vpLH[0], LOW); //make sure off
            }  // to else
          }
      
    }
  //------------------Right Foot----------------------
     if (header.from_node == 03) {

      if (vibePulse == true) {
      unsigned long timeElapsedRF = millis();
      if (dataPack.distance < 100) { // distance limit, increas to increase limit
            if (timeElapsedRF - timeAftRF >= dataPack.distance*5){  //create pulsing 
              timeAftRF = timeElapsedRF;
                   if (vibStateRF == LOW) {
                vibStateRF = HIGH;
              } else {
                vibStateRF = LOW;
              } // to else
             } // to if time of vib
         digitalWrite(vpRF[2], vibStateRF);  // output vib 
      } //if range limit
        else  { digitalWrite(vpRF[0], LOW); //make sure off
                digitalWrite(vpRF[1], LOW);
                digitalWrite(vpRF[2], LOW); 
      }  // to else

          } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpRF[2], ((100 - dataPack.distance) * 2.55)); 
            }  else  {
              digitalWrite(vpRF[2], LOW); //make sure off
            }  // to else
          }
      
    }
  //------------------Left Foot----------------------
     if (header.from_node == 04) {

      if (vibePulse == true) {
      unsigned long timeElapsedLF = millis();
      if (dataPack.distance < 100) { // distance limit, increas to increase limit
            if (timeElapsedLF - timeAftLF >= dataPack.distance*5){  //create pulsing 
              timeAftLF = timeElapsedLF;
                   if (vibStateLF == LOW) {
                vibStateLF = HIGH;
              } else {
                vibStateLF = LOW;
              } // to else
             } // to if time of vib
         digitalWrite(vpLF[2], vibStateLF);  // output vib 
      } //if range limit
        else  { digitalWrite(vpLF[0], LOW); //make sure off
                digitalWrite(vpLF[1], LOW);
                digitalWrite(vpLF[2], LOW);
      }  // to else

        } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpLF[2], ((100 - dataPack.distance) * 2.55)); 
            }  else  {
              digitalWrite(vpLF[2], LOW); //make sure off
            }  // to else
          }
      
    }
    
 } // to while loop 
  }else {       digitalWrite(vpRH[0], LOW); //make sure off
                digitalWrite(vpRH[1], LOW);
                digitalWrite(vpRH[2], LOW);
                digitalWrite(vpLH[0], LOW); //make sure off
                digitalWrite(vpLH[1], LOW);
                digitalWrite(vpLH[2], LOW);
                digitalWrite(vpRF[0], LOW); //make sure off
                digitalWrite(vpRF[1], LOW);
                digitalWrite(vpRF[2], LOW);
                digitalWrite(vpLF[0], LOW); //make sure off
                digitalWrite(vpLF[1], LOW);
                digitalWrite(vpLF[2], LOW);
  } // normally off

    
     //--- Serial write data
     Serial.print("Dist (cm): ");
     Serial.println(dataPack.distance);
 //    Serial.print(" Header data: ");
  //   Serial.print(header.from_node);
     
     

 delay(100);
}
