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
const uint16_t this_node = 03;   // either: 01(RH), 02(LH), 03(RF) or 04(LF)
const uint16_t master00 = 00;    // Address of the other node in Octal format

const unsigned long interval = 10;  //ms  // How often to send data to the other unit
unsigned long last_sent;            // When did we last send?


const int trigPin = 3;
const int echoPin = 2;

float duration, distance, timer;
int mes = 10;
int accel;



void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 

   //radio setup
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  
  
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

  // send data out every 10ms
   unsigned long now = millis();
    if (now - last_sent >= interval) {   // If it's time to send a data, send it!
      last_sent = now;
  
     //---Mag smoothing
    
    //Dampening the spikes, decrese damp for turer values
    magX_damp = magX_damp * 0.9 +  IMU.getMagX_uT() * 0.1;
    magY_damp = magY_damp * 0.9 + IMU.getMagY_uT() * 0.1;
  
    // --- Calculate the heading
    Heading = magY_damp;
    
    Heading2 = ((atan2(magX_damp, magY_damp)) * 180) / PI;
  
    
    if(Heading2 < 0) {
      Heading2+=360;
      Heading2 = 360-Heading2;
    }
  
    //--- Distance sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    timer = distance*5;
  
    accel = accel * 0.25+ IMU.getAccelY_mss() * 0.75; //smooth acc
    
    //---create data packet
    struct DataPack {
      int distance1 = round(distance)*1; //For the left multiply by 1.5 and delete for right, current issue
      int heading1 = Heading; //more persice
      int accel1 = abs(accel);
    };
    
    RF24NetworkHeader header(master00);   // (Address where the data is going)
    DataPack dataPack; // creates variabel, to call use dataPack.distance
    //send over radio
    network.write(header, &dataPack, sizeof(dataPack));
  }
  // print
  Serial.print("Distance (cm): ");
  Serial.print(distance);
  Serial.print(" Frequency (Hz): ");
  Serial.print(1/(distance*.01));
  Serial.print(" MagY: ");  
  Serial.print(magY_damp);
  Serial.print(" acc: ");  
  Serial.print(accel);
  Serial.print(" Heading: ");
  Serial.println(Heading2);
  
    
}
