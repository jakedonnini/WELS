


#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver(2000, 2, 4, 5); // pin 2

void setup()
{
  Serial.begin(9600); // Debugging only
  Serial.println("Setup");
  if (!driver.init())
    Serial.println("init failed");
}

void loop()
{
  //uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  int receivedData = 0;
  uint8_t buflen = sizeof(receivedData);

  //if (driver.recv(buf, &buflen)) // Non-blocking
  if (driver.recv((uint8_t*)receivedData, &buflen)) //if data is not an array, use &receivedData
  {
    for (byte i = 0; i < 3; i++)
    {
      Serial.println(receivedData);
     // Serial.print('\t');
     // Serial.println(receivedData[i],HEX);
    }
  }
}
