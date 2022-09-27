const int vibePin1 = 2;
const int echoPin1 = 3;
const int trigPin1 = 4; // Right hand

const int vibePin2 = 5;
const int echoPin2 = 6;
const int trigPin2 = 7; // Left hand

const int vibePin3 = 8;
const int echoPin3 = 9;
const int trigPin3 = 9; // Right foot

const int vibePin4 = 11;
const int echoPin4 = 12;
const int trigPin4 = 13; // Left foot

int vibeState1 = LOW;
int vibeState2 = LOW;
int vibeState3 = LOW;
int vibeState4 = LOW;

unsigned long previousMillis1 = 0;     // will store last time was updated
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long previousMillis4 = 0;

float duration1, distance1, timer1;
float duration2, distance2, timer2;
float duration3, distance3, timer3;
float duration4, distance4, timer4;



void setup() {
  Serial.begin(9600);
  pinMode(vibePin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT); // Right hand
  
  pinMode(vibePin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin2, OUTPUT); // Left hand
  
  pinMode(vibePin3, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT); // Right foot
  
  pinMode(vibePin4, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT); // Left foot

}

int rightHand() { //fuction for right hand
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1*.0343)/2;
  timer1 = distance1*5;
  
  unsigned long currentMillis1 = millis();  //how long in mils

  if (distance1 < 200) {
   //Attempt one
    if (currentMillis1 - previousMillis1 >= timer1) {
      // save the last time 
      previousMillis1 = currentMillis1;
  
    if (vibeState1 == LOW) {
      vibeState1 = HIGH;
    } else {
      vibeState1 = LOW;
    }
    
  } else { vibeState1 = LOW;}
  // Serial.print(currentMillis1 - previousMillis1);
  digitalWrite(vibePin1, vibeState1);
  }
}
//---------------------------------------------------------------------------------
int leftHand() { //fuction for left hand
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2*.0343)/2;
  timer2 = distance2*5;
  
  unsigned long currentMillis2 = millis();  //how long in mils

  if (distance2 < 200) {
   //Attempt one
    if (currentMillis2 - previousMillis2 >= timer2) {
      // save the last time 
      previousMillis2 = currentMillis2;
  
    if (vibeState2 == LOW) {
      vibeState2 = HIGH;
    } else {
      vibeState2 = LOW;
    }
    
  } else { vibeState2 = LOW;}
  
  digitalWrite(vibePin2, vibeState2);
  }
}
//---------------------------------------------------------------------------------
int rightFoot() { //fuction for right foot
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3*.0343)/2;
  timer3 = distance3*5;
  
  unsigned long currentMillis3 = millis();  //how long in mils

  if (distance3 < 200) {
   //Attempt one
    if (currentMillis3 - previousMillis3 >= timer3) {
      // save the last time 
      previousMillis3 = currentMillis3;
  
    if (vibeState3 == LOW) {
      vibeState3 = HIGH;
    } else {
      vibeState3 = LOW;
    }
    
  } else { vibeState3 = LOW;}
  
  digitalWrite(vibePin3, vibeState3);
  }
}
//---------------------------------------------------------------------------------
int leftFoot() { //fuction for left foot
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  
  duration4 = pulseIn(echoPin4, HIGH);
  distance4 = (duration4*.0343)/2;
  timer4 = distance4*5;
  
  unsigned long currentMillis4 = millis();  //how long in mils

  if (distance4 < 200) {
   //Attempt one
    if (currentMillis4 - previousMillis4 >= timer4) {
      // save the last time 
      previousMillis4 = currentMillis4;
  
    if (vibeState4 == LOW) {
      vibeState4 = HIGH;
    } else {
      vibeState4 = LOW;
    }
    
  } else { vibeState4 = LOW;}
  
  digitalWrite(vibePin4, vibeState4);
  }
}
void loop() {
  rightHand();
  leftHand();
  rightFoot();
  leftFoot();


  Serial.print(" Distance1: ");
  Serial.print(distance1);
  Serial.print(" Distance2: ");
  Serial.print(distance2);
  Serial.print(" Distance3: ");
  Serial.print(distance3);
  Serial.print(" Distance4: ");
  Serial.print(distance4);
  Serial.print(" Timer avg: ");
  Serial.println((timer1+timer2+timer3+timer4)/4);


}
