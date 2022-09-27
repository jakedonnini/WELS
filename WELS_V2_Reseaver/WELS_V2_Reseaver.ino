//This version uses the dim function to lower the intensity of the vibrations
//currently this doesn't have direction fuctionality.

 

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
int Heading;
float Heading2;

//--- radio
RF24 radio(7,8); // CE, CSN for mega
RF24Network network(radio);      // Include the radio in the network
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t rightHand01 = 01;      // Address of the other node in Octal format
const uint16_t leftHand012 = 02;
const uint16_t rightFoot04 = 03;      // 0X for child, 0XX for grand child, use only child 
const uint16_t leftFoot022 = 04;


//--- vib setup
int vpRH[] = {2, 3, 4}; //Pwm pins
int vpLH[] = {5, 6, 9}; 
int vpRF[] = {11, 10, 13};
int vpLF[] = {44, 45, 46};
int pinCount = 3;
int vibAngleRH, vibAngleLH, vibAngleRF, vibAngleLF;
int vibAngleRawRH, vibAngleRawLH, vibAngleRawRF, vibAngleRawLF;
int vibIntenseRH, vibIntenseLH, vibIntenseRF, vibIntenseLF;

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

//--- vib fuction 
int vibWrite(int angle, byte intes, int limb){
  //angle- 1 through 4 of where on the side it should vibrate
  //intesity(intes)- 1 through 255 in strangth
  //limb- from what limb does the data come from, just write header.from_node for now
    // Right Hand = 1
    // Left Hand = 2
    // Right Foot = 3
    // Left Foot = 4

  if (limb == 01) { //right hand
    analogWrite(vpRH[angle], intes);
  } else if (limb == 02) { //left hand
    analogWrite(vpLH[angle], intes);
  } else if (limb == 03) { //right foot
    analogWrite(vpRF[angle], intes);
  } else if (limb == 04) { //left foot
    analogWrite(vpLF[angle], intes);
  }else { //error
    Serial.println("limb error, no limb selected");
  }
}

void loop() {
  network.update(); //update network
  IMU.readSensor(); //read sensor data
  
    //---Mag smoothing
  magPitch = -IMU.getGyroY_rads() * DEG_TO_RAD;
  magRoll = IMU.getGyroX_rads() * DEG_TO_RAD;

  //angle compensation
  //magX_hor = IMU.getMagX_uT() * cos(magPitch) + IMU.getMagY_uT() * sin(magRoll) * sin(magPitch) - IMU.getMagZ_uT() * cos(magRoll) * sin(magPitch);
  //magY_hor = IMU.getMagY_uT() * cos(magRoll) + IMU.getMagZ_uT() * sin(magRoll);

  magX_hor =  IMU.getMagX_uT();
  magY_hor = IMU.getMagY_uT();

  Heading2 = ((atan2(magX_hor,magY_hor)) * 180) / PI
  
  //Dampening the spikes, decrese damp for turer values
  magX_damp = magX_damp * 0.9 + magX_hor * 0.1;
  magY_damp = magY_damp * 0.9 + magY_hor * 0.1;

  // --- Calculate the heading
  Heading = atan2(magX_damp, magY_damp) * RAD_TO_DEG;  // Magnetic North

  DataPack dataPack; // declare data pack
  RF24NetworkHeader header;

  //--- make radio connection
  if (network.available()) { 
   while (network.available()) { // if radio is found

      network.read(header, &dataPack, sizeof(dataPack));   //read radio

        //--- Vibe angle decode
      if (vibAngleRawRH < 0) {
        vibAngleRH == 0;
      } else if ( 0 <= vibAngleRawRH < 10) {
        vibAngleRH == 1;
      } else if ( 10 <= vibAngleRawRH < 20) {
        vibAngleRH == 2;
      } else {
        vibAngleRH == 0;
      } 


      if (header.from_node == 01) {//right hand
            vibIntenseRH = 255 - (dataPack.distance *2.55);           // switched distance calculation form exponetial to linear
            vibAngleRawRH = abs(Heading) - abs(dataPack.magValIn);
            if (dataPack.distance < 100) {                            // dsitance limit
          vibWrite(vibAngleRH, vibIntenseRH, 01);                  // write right vib functon
        } else {
          vibWrite(vibAngleRH, 0, 01);   
      }
      }

      if (header.from_node == 02) {//left Hand
             vibIntenseLH = 255 - (dataPack.distance *2.55);           // switched distance calculation form exponetial to linear
            if (dataPack.distance < 100) {                            // dsitance limit
          vibWrite(vibAngleLH, vibIntenseLH, 02);                  // write right vib functon
        } else {
          vibWrite(vibAngleLH, 0, 02);   
      }
      }

      if (header.from_node == 03) {//left Hand
             vibIntenseRF = 255 - (dataPack.distance *2.55);           // switched distance calculation form exponetial to linear
            if (dataPack.distance < 100) {                            // dsitance limit
          vibWrite(vibAngleRF, vibIntenseRF, 03);                  // write right vib functon
        } else {
          vibWrite(vibAngleRF, 0, 03);   
      }
      }

      if (header.from_node == 04) {//left Hand
             vibIntenseLF = 255 - (dataPack.distance *2.55);           // switched distance calculation form exponetial to linear
            if (dataPack.distance < 100) {                            // dsitance limit
          vibWrite(vibAngleLF, vibIntenseLF, 04);                  // write right vib functon
        } else {
          vibWrite(vibAngleLF, 0, 04);   
      }
      }
        
   }
  } else {
    vibWrite(vibAngleRH, 0, 01);
    vibWrite(vibAngleLH, 0, 012);
    vibWrite(vibAngleRF, 0, 04); 
    vibWrite(vibAngleLF, 0, 022);
  }
    
     //--- Serial write data
     Serial.print("Dist (cm): ");
     Serial.print(dataPack.distance);
     Serial.print(" Header data: ");
     Serial.print(header.from_node);
     Serial.print(" IM: ");
     Serial.print(dataPack.magValIn); // incoming mag data
     Serial.print(" Heading degree: ");
     Serial.print(Heading2);           // internal mag data
     Serial.print(" Angle: ");
     Serial.print(vibAngleRH);
     Serial.print(" vib int: ");
     Serial.println(vibAngleRawRH);
     
 delay(100);
}
