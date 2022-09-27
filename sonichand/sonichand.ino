const int trigPin = 4;
const int echoPin = 3;
const int vibPin = 5;
int vibState = LOW;



unsigned long previousMillis = 0;        // will store last time was updated

float duration, distance, timer;


void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(vibPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  timer = distance*5;
  
  unsigned long currentMillis = millis();  //how long in mils
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" Timer: ");
  Serial.print(timer);
 Serial.print(" Mil Dif: ");
 Serial.println(currentMillis - previousMillis);

  if (distance < 200) {
   //Attempt one
    if (currentMillis - previousMillis >= timer) {
      // save the last time 
      previousMillis = currentMillis;
  
    if (vibState == LOW) {
      vibState = HIGH;
    } else {
      vibState = LOW;
    }
    
  } else { vibState = LOW;}
  
digitalWrite(vibPin, vibState);

 
} 


 delay(100);
}


