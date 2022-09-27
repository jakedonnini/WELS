// NodeMCU-Blynk.ino
// For M&TSI 2021

#define BLYNK_PRINT Serial

// include libraries required for Blynk and NodeMCU communication
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// you should get Auth Token in the Blynk App.
// go to the Project Settings (nut icon).
char auth[] = "insert auth code here";

// your WiFi credentials.
// set password to "" for open networks.
char ssid[] = "insert Wi-Fi network name here";
char pass[] = "insert Wi-Fi network password here";

// create variable of type BlynkTimer, see more details below
BlynkTimer timer;

// these correspond to the pins on your NodeMCU
#define trigPin 14    // D5 in Node MCU
#define echoPin 12    // D6 in Node MCU
const int greenPin = 5; // D1 in Node MCU
const int redPin = 4;   // D2 in Node MCU

void setLED(int distance) {
  if (distance >= 0 && distance < 12) {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
  } else if (distance >= 12 && distance < 40) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
  } else if (distance>=40) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
  }
}

long getDistance()
{
  long duration, distance;
  
  // clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // calculating the distance
  distance = (duration / 2) / 29.1;

  // changes the LED color based on the distance calculated previously
  setLED(distance);
  
  // the following code can be useful in order to debug any problems with your ping sensor
  // in order to use it, uncomment the code below
  //  Serial.print("Duration: ");
  //  Serial.print(duration);
  //  Serial.print(" Distance: ");
  //  Serial.println(distance);
  
  return distance;
}

void myTimerEvent()
{
  float currentDistance = getDistance(); // try not to send >10 values/secondz
  Blynk.virtualWrite(V5, currentDistance); // send data to app
}

void setup()
{
  // sets up pins
  pinMode (trigPin , OUTPUT );
  pinMode (echoPin , INPUT );
  pinMode (redPin, OUTPUT);
  pinMode (greenPin, OUTPUT);
  
  // opens serial monitor at 9600 baud
  Serial.begin(9600);

  // starts the connection with Blynk using the data provided at the top (Wi-Fi connection name, password, and auth token)
  Blynk.begin(auth, ssid, pass);

  // a timer function which is called every 1000 millisecond. Note that it calls the function myTimerEvent, which in turn send the currentDistance to the Blynk server
  timer.setInterval(1000L, myTimerEvent); // setup a function to be called every second
}

void loop()
{
  // runs the code
  Blynk.run();
  timer.run();
}
