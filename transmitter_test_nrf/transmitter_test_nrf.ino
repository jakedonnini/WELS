
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresse= "00001";


void setup() {
  radio.begin();
  radio.openWritingPipe(addresse);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  delay(5);
  int angleValue = 0;
  angleValue++;
  radio.write(&angleValue, sizeof(angleValue));

}
