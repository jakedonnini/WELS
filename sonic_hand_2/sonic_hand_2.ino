#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

const int trigPin = 4;
const int echoPin = 3;

float duration, distance, timer;

RH_ASK driver;

void setup()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);    // Debugging only
  if (!driver.init())
    Serial.println("init failed");
}

void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  timer = distance*5;

  Serial.print("Distance: ");
  Serial.println(distance);
  
      
    char buf[10];
    itoa(distance,buf, 10); 
    driver.send((uint8_t *)buf, strlen(buf));
    driver.waitPacketSent();
    delay(10);
}
