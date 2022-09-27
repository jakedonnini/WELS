#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001"; 
const int vibePin = 2;
int vibState = LOW;
int fCount = 0;
unsigned long timeAft = 0;


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  pinMode(vibePin, OUTPUT);

}
void loop() {
  
  if (radio.available()) {
   while (radio.available()) {
    int distance = 0;
    radio.read(&distance, sizeof(distance));
    
    unsigned long timeElapsed = millis();
   
    
    if (distance < 100) {
     
          if (timeElapsed - timeAft >= distance*5){
            timeAft = timeElapsed;
                 if (vibState == LOW) {
              vibState = HIGH;
            } else {
              vibState = LOW;
            }
           }
         
       digitalWrite(vibePin, vibState);


       
    }
      else  { digitalWrite(vibePin, LOW);
    }
     Serial.print("Distance (cm): ");
     Serial.print(distance);
     Serial.print(" Frequency (Hz): ");
     Serial.println(1/(distance*.01)); //frequency calc
 } 
  }else { digitalWrite(vibePin, LOW);} 

 
 delay(100);
  
}
