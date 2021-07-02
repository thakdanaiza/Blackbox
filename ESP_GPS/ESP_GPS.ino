#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define RXPin (16)
#define TXPin (17)

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial ss(2);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);

}

void loop()
{

  if(ss.available() > 0)
  {
    gps.encode(ss.read());
    if (gps.time.isUpdated())
    {
      if ((gps.time.hour() + 7) < 10) Serial.print(F("0"));
      Serial.print((gps.time.hour() + 7));
      Serial.print(":");
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(":");
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.println(gps.time.second());
      delay(1000);
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}
