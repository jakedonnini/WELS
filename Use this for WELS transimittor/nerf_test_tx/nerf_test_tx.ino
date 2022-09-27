#include <RF24Network.h>
#include <RF24Network_config.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <math.h>

RF24 radio(7, 8); // CE, CSN
RF24Network network(radio);      // Include the radio in network
const uint16_t this_node = 01;   // either: 01(RH), 02(LH), 03(RF) or 04(LF)
const uint16_t master00 = 00;    // Address of the other node in Octal format

const unsigned long interval = 10;  //ms  // How often to send data to the other unit
unsigned long last_sent;            // When did we last send?

const int trigPin = 3;
const int echoPin = 2;

float duration, distance, timer;


void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
}

void loop() {
  network.update();

  
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
    int distance1 = round(distance);
    int heading1 = 0;
  };
  
  RF24NetworkHeader header(master00);   // (Address where the data is going)
  DataPack dataPack; // creates variabel, to call use dataPack.distance
  //send over radio
  network.write(header, &dataPack, sizeof(dataPack));
  
   
  delay(10);
}
