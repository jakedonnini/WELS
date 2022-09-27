#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
#include "MPU9250.h"
#include <SPI.h>
#include <nRF24L01.h>

#include <RF24.h>

RF24 radio(7,8); // CE, CSN for mega
//const byte address[6] = "00001"; 
RF24Network network(radio);      // Include the radio in the network
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t rightHand01 = 01;      // Address of the other node in Octal format
const uint16_t rightFoot04 = 02;
const uint16_t leftHand012 = 03;
const uint16_t leftFoot022 = 04;

//--- vib setup
unsigned long timeAft = 0;
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
   // Serial.println("limb error, no limb selected");
  }
}

void loop() {
  network.update(); //update network

    DataPack dataPackRH; // declare data pack
    DataPack dataPackLH;
    DataPack dataPackRF;
    DataPack dataPackLF;
    RF24NetworkHeader header;

  //--- make radio connection
  if (network.available()) { 
   while (network.available()) { // if radio is found
     

  

    network.read(header, &dataPackRH, sizeof(dataPackRH));

     if (header.from_node == 01) {
            vibIntenseRH = 255 - (dataPackRH.distance *2.55);           // switched distance calculation form exponetial to linear
            if (dataPackRH.distance < 100) {                            // dsitance limit
          vibWrite(vibAngleRH, vibIntenseRH, 01);                  // write right vib functon
        } else {
          vibWrite(vibAngleRH, 0, 01);   
      }
      }

      if (header.from_node == 02) {
             vibIntenseLH = 255 - (dataPackRH.distance *2.55);           // switched distance calculation form exponetial to linear
            if (dataPackRH.distance < 100) {                            // dsitance limit
          vibWrite(vibAngleLH, vibIntenseLH, 02);                  // write right vib functon
        } else {
          vibWrite(vibAngleLH, 0, 02);   
      }
      }

   } 

   
  } else {
    vibWrite(vibAngleRH, 0, 01);
    vibWrite(vibAngleLH, 0, 02);
    vibWrite(vibAngleRF, 0, 03); 
    vibWrite(vibAngleLF, 0, 04);

    dataPackRH.distance = 0;
    dataPackLH.distance = 0;
    dataPackRF.distance = 0;
    dataPackLF.distance = 0;
  }
     Serial.print("Dist (cm): ");
     Serial.print(dataPackRH.distance);
     Serial.print(" ");
     Serial.print(dataPackLH.distance);
     Serial.print(" ");
     Serial.print(dataPackRF.distance);
     Serial.print(" ");
     Serial.print(dataPackLF.distance);
     Serial.print(" Header data: ");
     Serial.println(header.from_node);

     delay(100);
}
