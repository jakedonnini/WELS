
#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

RH_ASK driver(2000, 4, 2, 5); // pin 2
char b[5];
String str;




void setup() {
  Serial.begin(9600);
  if (!driver.init())
      Serial.println("TX init failed"); 
}

void loop() {
  uint16_t analog = 10;


 
   analog++;
  
   
  
  int data = analog;
 


  driver.send((uint8_t*)data, 3); //6 == 3*sizeof(data[0])
 
  driver.waitPacketSent(); 
  
 
     
  delay(200);
}
