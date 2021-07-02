#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP085_U.h>
#include <TinyGPS++.h>
#include "HardwareSerial.h";

LSM303 compass;

char report[120];

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void setup() {
  Serial.begin(9600);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  gyro.enableAutoRange(true);
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while (1);
  }
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void loop() {
  compass.read();
  
  snprintf(report, sizeof(report), "A: %6d %6d %6d    M: %6d %6d %6d",
           compass.a.x, compass.a.y, compass.a.z,
           compass.m.x, compass.m.y, compass.m.z);
  Serial.println(report);
  
  //appendFile(SD, "/Datalog.txt", (compass.a.x));
  sensors_event_t event1;
  gyro.getEvent(&event1);

  Serial.print("X: "); Serial.print(event1.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event1.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event1.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  sensors_event_t event2;
  bmp.getEvent(&event2);
  Serial.print("Pressure:    ");
  Serial.print(event2.pressure);
  Serial.println(" hPa");
  float temperature;
  bmp.getTemperature(&temperature);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  Serial.print("Altitude:    ");
  Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                      event2.pressure));
  Serial.println(" m");
  Serial.println("");
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
  delay(1000);
}
