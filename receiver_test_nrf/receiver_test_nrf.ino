
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(7, 8); // CE, CSN
const byte addresse = "00001";


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, addresse); 
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  delay(5);
  
  if ( radio.available()) {
    while (radio.available()) {
      int angleV = 0;
      radio.read(&angleV, sizeof(angleV));
      Serial.println(angleV);
    }
  }
}
