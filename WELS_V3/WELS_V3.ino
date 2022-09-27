

// even echo, odd trig
const int e1 = 22;
const int t1 = 23;
const int e2 = 24;
const int t2 = 25; 
const int e3 = 26;
const int t3 = 27;
const int e4 = 28;
const int t4 = 29;
const int e5 = 30;
const int t5 = 31;
const int e6 = 32;
const int t6 = 33;
const int e7 = 34;
const int t7 = 35;
const int e8 = 36;
const int t8 = 37;
const int e9 = 38;
const int t9 = 39;
const int e10 = 40;
const int t10 = 41;

float duration, distance, timer;
float duration1, distance1, timer1;
float duration2, distance2, timer2;
float duration3, distance3, timer3;
float duration4, distance4, timer4;
float duration5, distance5, timer5;
float duration6, distance6, timer6;
float duration7, distance7, timer7;
float duration8, distance8, timer8;
float duration9, distance9, timer9;
float duration10, distance10, timer10;



void setup() {
  Serial.begin(9600);

  pinMode(t1, OUTPUT);
  pinMode(e1, INPUT);
  pinMode(t2, OUTPUT);
  pinMode(e2, INPUT);
  pinMode(t3, OUTPUT);
  pinMode(e3, INPUT);
  pinMode(t4, OUTPUT);
  pinMode(e4, INPUT);
  pinMode(t5, OUTPUT);
  pinMode(e5, INPUT);
  pinMode(t6, OUTPUT);
  pinMode(e6, INPUT);
  pinMode(t7, OUTPUT);
  pinMode(e7, INPUT);
  pinMode(t8, OUTPUT);
  pinMode(e8, INPUT);
  pinMode(t9, OUTPUT);
  pinMode(e9, INPUT);
  pinMode(t10, OUTPUT);
  pinMode(e10, INPUT);
 
}

void SonarSensor(int trigPin,int echoPin){
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
//duration = pulseIn(echoPin, HIGH);
distance = (duration*.0343)/2;

}

void loop() {
  //dist 1
  
  SonarSensor(t1,e1);
  distance1 = distance;
  timer1 = distance1*5;
 
  //dist 2
  SonarSensor(t2,e2);
  distance2 = distance;
  timer2 = distance2*5;

  //dist 3
  SonarSensor(t3,e3);
  distance3 = distance;
  timer3 = distance3*5;

  //dist 4
  SonarSensor(t4,e4);
  distance4 = distance;
  timer4 = distance4*5;

  //dist 5
  SonarSensor(t5,e5);
  distance5 = distance;
  timer5 = distance5*5;

  //dist 6
  SonarSensor(t6,e6);
  distance6 = distance;
  timer6 = distance6*5;

  //dist 7
  SonarSensor(t7,e7);
  distance7 = distance;
  timer7 = distance7*5;

  //dist 8
  SonarSensor(t8,e8);
  distance8 = distance;
  timer8 = distance8*5;

  //dist 9
  SonarSensor(t9,e9);
  distance9 = distance;
  timer9 = distance9*5;

   //dist 10
  SonarSensor(t10,e10);
  distance10 = distance;
  timer10 = distance10*5;
  
  
  
  // Serial out 
  Serial.print(distance1);
  Serial.print("\t");
  Serial.print(distance2);
  Serial.print("\t");
  Serial.print(distance3);
  Serial.print("\t");
  Serial.print(distance4);
  Serial.print("\t");
  Serial.print(distance5);
  Serial.print("\t");
  Serial.print(distance6);
  Serial.print("\t");
  Serial.print(distance7);
  Serial.print("\t");
  Serial.print(distance8);
  Serial.print("\t");
  Serial.print(distance9);
  Serial.print("\t");
  Serial.println(distance10);
  
  delay(10);
}
