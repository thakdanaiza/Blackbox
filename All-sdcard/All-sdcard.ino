#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <LSM303.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

void Getaccel(void)
{
  mpu_accel = mpu.getAccelerometerSensor();
  sensors_event_t accel;
  mpu_accel->getEvent(&accel);

  /*
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
  */

  File file = SD.open("/Data.txt", FILE_APPEND);
  file.print("/");
  file.print(accel.acceleration.x);
  file.print("/");
  file.print(accel.acceleration.y);
  file.print("/");
  file.print(accel.acceleration.z);
  file.close();

}
void Getgyro(void)
{
  mpu_gyro = mpu.getGyroSensor();
  sensors_event_t gyro;
  mpu_gyro->getEvent(&gyro);
  /*
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();
  */
  File file = SD.open("/Data.txt", FILE_APPEND);
  file.print("/");
  file.print(gyro.gyro.x);
  file.print("/");
  file.print(gyro.gyro.y);
  file.print("/");
  file.println(gyro.gyro.z);
  file.close();
}


LSM303 compass;

char report[80];

void Getmagnetic(void)
{
  compass.read();
  /*
    snprintf(report, sizeof(report), " M: %6d %6d %6d",
             compass.m.x, compass.m.y, compass.m.z);
    Serial.println(report);
  */
  File file = SD.open("/Data.txt", FILE_APPEND);
  file.print("/");
  file.print(compass.m.x);
  file.print("/");
  file.print(compass.m.y);
  file.print("/");
  file.print(compass.m.z);
  file.close();
}
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(2);
void GetGPS(void)
{
  File file = SD.open("/Data.txt", FILE_APPEND);
  //Serial.print(F("Location: "));

  if (gps.location.isValid())
  {
    /*
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    */
    file.print("/");
    file.print(gps.location.lat(), 6);
    file.print("/");
    file.print(gps.location.lng(), 6);
  }
  else
  {
    //Serial.println(F("INVALID"));
    file.print("/");
    file.print("o");
    file.print("/");
    file.print("o");
  }
  file.close();
}
void GetDate(void)
{
  File file = SD.open("/Data.txt", FILE_APPEND);
  //Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {

    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());

    file.print("/");
    file.print(gps.date.day());
    file.print("/");
    file.print(gps.date.month());
    file.print("/");
    file.print(gps.date.year());
  }
  else
  {
    //Serial.println(F("INVALID"));
    file.print("/");
    file.print("o");
    file.print("/");
    file.print("o");
    file.print("/");
    file.print("o");
  }
  file.close();
}
void GetTime(void)
{
  File file = SD.open("/Data.txt", FILE_APPEND);
  // Serial.print(F(" "));
  if (gps.time.isValid())
  {
    /*
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));

        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());

        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
    */

    if (gps.time.hour() < 10) file.print(F("0"));
    file.print(F("/"));
    file.print(gps.time.hour());

    Serial.print(gps.time.hour());
    Serial.print(F(":"));


    if (gps.time.minute() < 10) Serial.print(F("0"));
    file.print(F("/"));
    file.print(gps.time.minute());
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    file.print(F("/"));
    file.print(gps.time.second());
    Serial.print(gps.time.second());
    Serial.print(F("."));
  }
  else
  {
    //Serial.print(F("INVALID"));
    file.print("/");
    file.print("o");
    file.print("/");
    file.print("o");
    file.print("/");
    file.print("o");
  }
  file.close();

}

void setup() {

  Serial.begin(9600);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    //return;
  }

  if (!SD.begin())
  {
    Serial.println("Card Mount Failed");
    // return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    //return;
  }
  compass.init();
  compass.enableDefault();
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);

  File file = SD.open("/Data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    // return;
  }
  pinMode(13, OUTPUT);
  if (file.print("test")) {
    //LED ON
    digitalWrite(13, HIGH);
    delay(5000);
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
  } else {
    digitalWrite(13, LOW);
    //LED OFF
    return;
  }
  file.close();

}

void loop() {
  
  unsigned long start = millis ();
  Serial.print("1");

  while (ss.available() > 0) {

    if (gps.encode(ss.read())) {
      GetTime(); //1/2/3
      GetDate(); //1/2/3
      GetGPS(); //1/2
      break;
    }
    else {
      File file = SD.open("/Data.txt", FILE_APPEND);
      file.print("0/0/0/0/0/0/0/0/");
      file.close();
      Serial.print("no");
      break;
    }
  }
  Serial.print("2");
  //Getmagnetic(); //1/2/3
  Serial.print("3");
  Getaccel(); //1/2/3
  Serial.println("4");
  Getgyro(); //1/2/3


  delay(1000);
}
