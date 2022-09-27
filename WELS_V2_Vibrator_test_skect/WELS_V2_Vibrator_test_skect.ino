

//for testing the vibrators, it just sycles through


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
bool vibePulse = true;

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

 
  
}


void loop() {

  
   for (int i; i < 3; i++) { //test all vibes on RH side
    digitalWrite(vpRH[i], HIGH);
    delay(2500);
    digitalWrite(vpRH[i], LOW);
    delay(2500);
   }

   for (int i; i < 3; i++) { //test all vibes on LH side
    digitalWrite(vpLH[i], HIGH);
    delay(2500);
    digitalWrite(vpLH[i], LOW);
    delay(2500);
   }

   for (int i; i < 3; i++) { //test all vibes on RF side
    digitalWrite(vpRF[i], HIGH);
    delay(2500);
    digitalWrite(vpRF[i], LOW);
    delay(2500);
   }

   for (int i; i < 3; i++) { //test all vibes on LF side
    digitalWrite(vpLF[i], HIGH);
    delay(2500);
    digitalWrite(vpLF[i], LOW);
    delay(2500);
   }

   delay(10000);
}
