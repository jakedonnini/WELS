
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver(2000, 2, 4, 5); // pin 2

void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {

      String rcv;
  
      for (int i = 0; i < buflen; i++) {
        rcv += (char)buf[i];
      }
  
      Serial.println(rcv);
    }    
}
