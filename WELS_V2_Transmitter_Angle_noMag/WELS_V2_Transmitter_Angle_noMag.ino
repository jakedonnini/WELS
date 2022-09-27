//This verson uses only the gyro to determin position.
//It also is the only version to feature the stand alone code that should be standard in all future versions.
//added double gyro

#include <MPU9250.h>
#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
#include "MPU9250.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <math.h>

//mag sensor
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
const int MPU_addr=0x68;
const int MPU2_addr=0x69;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t AcX2,AcY2,AcZ2,Tmp2,GyX2,GyY2,GyZ2;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;

double x2;
double y2;
double z2;

int status;
float magPitch, magRoll;
float magX_hor, magY_hor;
float magX_damp, magY_damp;
int Heading; 
float Heading2;

//radio
RF24 radio(7, 8); // CE, CSN
RF24Network network(radio);      // Include the radio in network
const uint16_t this_node = 02;   // either: 01(RH), 02(LH), 03(RF) or 04(LF)
const uint16_t master00 = 00;    // Address of the other node in Octal format

const unsigned long interval = 10;  //ms  // How often to send data to the other unit
unsigned long last_sent;            // When did we last send?

//vibe pin setup
unsigned long timeAft = 0;
int vibState = LOW;

const int vibPin = 4;
const int trigPin = 3; //3
const int echoPin = 2; //2

float duration, distance, timer;
int mes = 10;
int accel;
int message;



void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(vibPin, OUTPUT);
 

   //radio setup
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);

  //wire to both gyros
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU2_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  

}

void loop() {
  //read the to gyros
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  
  Wire.beginTransmission(MPU2_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU2_addr,14,true);
  AcX2=Wire.read()<<8|Wire.read();
  AcY2=Wire.read()<<8|Wire.read();
  AcZ2=Wire.read()<<8|Wire.read();

  network.update();

  
   //---Interprit the incoming data of a 
  
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);

    int xAng2 = map(AcX2,minVal,maxVal,-90,90);
    int yAng2 = map(AcY2,minVal,maxVal,-90,90);
    int zAng2 = map(AcZ2,minVal,maxVal,-90,90);

       x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

       x2= RAD_TO_DEG * (atan2(-yAng2, -zAng2)+PI);
       y2= RAD_TO_DEG * (atan2(-xAng2, -zAng2)+PI);
       z2= RAD_TO_DEG * (atan2(-yAng2, -xAng2)+PI);

  // --- Calculate the heading
  

  //--- Distance sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  timer = distance*5;

  
  //---create data packet
  struct DataPack {
    int distance1 = round(distance)*1; //For the left multiply by 1.5 and delete for right, current issue
    int AngleX = x; 
    int AngleY = y;
    int AngleZ = z;
    int AngleX2 = x2; 
    int AngleY2 = y2;
    int AngleZ2 = z2;
  };

  

  RF24NetworkHeader header(master00);   // (Address where the data is going)
    DataPack dataPack; // creates variabel, to call use dataPack.distance
    //send over radio
   network.write(header, &dataPack, sizeof(dataPack));
   network.read(header, &message, sizeof(message));   //read radio

  unsigned long timeElapsed = millis(); // record time for vibe pulses
  
  if (message == 0) { //if message not 0 then it is not connected to it switches to stand alone
   Serial.print("Network Scr");
   if (round(distance) < 100) { // distance limit, increas to increase limit
            if (timeElapsed - timeAft >= round(distance)){  //create pulsing 
              timeAft = timeElapsed;
                   if (vibState == LOW) {
                vibState = HIGH;
              } else {
                vibState = LOW;
              } // to else
             } // to if time of vib
         digitalWrite(vibPin, vibState);  // output vib 
      } //if range limit
     else  { digitalWrite(vibPin, LOW); } //make sure off 
  } else {
    digitalWrite(vibPin, LOW);
  }
  
  // print
  Serial.print("Distance (cm): ");
  Serial.print(distance);
  Serial.print(" Frequency (Hz): ");
  Serial.print(1/(distance*.01));
     Serial.print("AngleX= ");
     Serial.print(x);
     Serial.print("\t");
     Serial.print("AngleX2= ");
     Serial.println(x2);

     Serial.print("AngleY= ");
     Serial.print(y);
     Serial.print("\t");
     Serial.print("AngleY2= ");
     Serial.println(y2);

     Serial.print("AngleZ= ");
     Serial.print(z);
     Serial.print("\t");
     Serial.print("AngleZ2= ");
     Serial.println(z2);
  
    
}
