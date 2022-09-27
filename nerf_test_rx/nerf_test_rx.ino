#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
const int vibePin = 2;
const int buzPin = 5;
int vibState = LOW;
int vibLevel = 0;
const long int1 = 2000;
const long int2 = 1500;
const long int3 = 1000;
const long int4 = 500;

unsigned long previousMillis = 0;        // will store last time was updated
float timer;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
  pinMode(vibePin, OUTPUT);
  pinMode(buzPin, OUTPUT);

  digitalWrite(buzPin, HIGH);
  delay(100);
  digitalWrite(buzPin, LOW);
}
void loop() {

  

  //unsigned long currentMillis = millis();  //how long in mils
  
  if (radio.available()) {
   while (radio.available()) {
    int distance = 0;
    radio.read(&distance, sizeof(distance));
    
    unsigned long timeElapsed = millis();

    
    if (distance < 100) {
      if (distance < 100 && distance > 75) { // 200-150 away pulse every sec
        
        unsigned long timeAft = 0;
          if (timeElapsed - timeAft >= int1){
            timeAft = timeElapsed;
                 if (vibState == LOW) {
              vibState = HIGH;
            } else {
              vibState = LOW;
            }
           }
       digitalWrite(vibePin, vibState);
       vibLevel = 1;
      }else if (distance < 75 && distance > 50) { //150-100 away pulse every half sec
        
        unsigned long timeAft1 = 0;
          if (timeElapsed - timeAft1 >= int2){
           timeAft1 = timeElapsed;
                 if (vibState == LOW) {
              vibState = HIGH;
            } else {
              vibState = LOW;
            }
         
       }
        digitalWrite(vibePin, vibState);
       vibLevel = 2;
      } else if (distance < 50 && distance > 25) { //100-50 away pulse every quarter sec
         
        unsigned long timeAft2 = 0;
          if (timeElapsed - timeAft2 >= int3){
           timeAft2 = timeElapsed;
                 if (vibState == LOW) {
              vibState = HIGH;
            } else {
              vibState = LOW;
            }
          
       }
       digitalWrite(vibePin, vibState);
       vibLevel = 3;
      } else if (distance < 25) { //50-0 away pulse every quarter sec
         
        unsigned long timeAft3 = 0;
          if (timeElapsed - timeAft3 >= int4){
          timeAft3 = timeElapsed;
                 if (vibState == LOW) {
              vibState = HIGH;
            } else {
              vibState = LOW;
            }
         
       }
       digitalWrite(vibePin, vibState);
       vibLevel = 4;
      } else { digitalWrite(vibePin, LOW);
      vibLevel = 0;
      }
    } else { digitalWrite(vibePin, LOW);
    vibLevel = 0;
    }

     //digitalWrite(vibePin, vibState);
    
    Serial.print("Distance: ");
     Serial.print(distance);
     Serial.print(" Timer: ");
     Serial.print(timer);
     Serial.print(" time Elapsed ");
     Serial.print(timeElapsed);
     Serial.print(" vibe level ");
     Serial.println(vibLevel);



   
  }
 } else { digitalWrite(vibePin, LOW);} 
 

 
 delay(100);
}
