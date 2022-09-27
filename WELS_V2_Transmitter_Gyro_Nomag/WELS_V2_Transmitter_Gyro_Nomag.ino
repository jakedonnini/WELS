//This verson uses only the gyro to determin position.
//It also is the only version to feature the stand alone code that should be standard in all future versions.


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
MPU9250 IMU(Wire,0x68);
int status;
float magPitch, magRoll;
float magX_hor, magY_hor;
float magX_damp, magY_damp;
int Heading; 
float Heading2;

//radio
RF24 radio(7, 8); // CE, CSN
RF24Network network(radio);      // Include the radio in network
const uint16_t this_node = 01;   // either: 01(RH), 02(LH), 03(RF) or 04(LF)
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
  
  IMU.calibrateMag();
  
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
  IMU.readSensor(); //read sensor data
  network.update();

  
   //---Mag smoothing
  
  //Dampening the spikes, decrese damp for turer values
  magX_damp = magX_damp * 0.9 +  IMU.getMagX_uT() * 0.1;
  magY_damp = magY_damp * 0.9 + IMU.getMagY_uT() * 0.1;

  // --- Calculate the heading
  
  
  Heading = ((atan2(magX_damp, magY_damp)) * 180) / PI;
  Heading2 = ((IMU.getGyroX_rads()) * 180) / PI; //use gyro instead

  //--- Distance sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  timer = distance*5;

  accel = accel * 0.25+ IMU.getAccelX_mss() * 0.75; //smooth acc
  
  //---create data packet
  struct DataPack {
    int distance1 = round(distance)*1; //For the left multiply by 1.5 and delete for right, current issue
    int heading1 = round(Heading2); //more persice
    int accel1 = abs(accel);
    int magOut = Heading;
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
  //Serial.print(distance);
  Serial.print(" Frequency (Hz): ");
  Serial.print(1/(distance*.01));
  Serial.print(" MagY: ");  
  Serial.print(magY_damp);
  Serial.print(" acc: ");  
  Serial.print(accel);
  Serial.print(" ping: ");
  Serial.println(message);
  
    
}
