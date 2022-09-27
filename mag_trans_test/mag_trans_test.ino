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


void setup() {
  Serial.begin(9600);
  SPI.begin();

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

 magPitch = -IMU.getGyroY_rads() * DEG_TO_RAD;
  magRoll = IMU.getGyroX_rads() * DEG_TO_RAD;

  //angle compensation
  //magX_hor = IMU.getMagX_uT() * cos(magPitch) + IMU.getMagY_uT() * sin(magRoll) * sin(magPitch) - IMU.getMagZ_uT() * cos(magRoll) * sin(magPitch);
 // magY_hor = IMU.getMagY_uT() * cos(magRoll) + IMU.getMagZ_uT() * sin(magRoll);

 magX_hor =  IMU.getMagX_uT();
 magY_hor = IMU.getMagY_uT();
  
  //Dampening the spikes, decrese damp for turer values
  magX_damp = magX_damp * 0.9 + magX_hor * 0.1;
  magY_damp = magY_damp * 0.9 + magY_hor * 0.1;

  magPitch = magPitch * 0.9 + IMU.getGyroY_rads() * 0.1;
  magRoll = magRoll * 0.9 + IMU.getGyroX_rads() * 0.1;

  // --- Calculate the heading
  Heading = atan2(magX_damp, magY_damp);  // Magnetic North

  //Serial.print(" MagX: ");  
 // Serial.print(IMU.getMagX_uT());
  Serial.print(" Magy: ");  
  Serial.print(magY_damp);
  Serial.print(" gryo Y: ");  
  Serial.print((magPitch * 180/PI));
  Serial.print(" gryo X: ");  
  Serial.println((magRoll * 180/PI));

  /*
  Serial.print(" MagX damp: ");  
  Serial.print(magX_damp);
  Serial.print(" Magy damp: ");  
  Serial.print(magY_damp);
  Serial.print(" Heading: ");
  Serial.println(Heading);
  */
  
 
}
