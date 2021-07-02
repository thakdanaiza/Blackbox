#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define RXPin (16)
#define TXPin (17)

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

// The TinyGPS++ object
TinyGPSPlus gps;
HardwareSerial ss(2);
static const uint32_t GPSBaud = 9600;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

int x;

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Wire.begin();

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
  File file = SD.open("/Data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    // return;
  }

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  if (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      /* Update all the values */
      while (i2cRead(0x3B, i2cData, 14));
      accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
      accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
      accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
      tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
      gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
      gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
      gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();

      // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
      // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
      // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(accY, accZ) * RAD_TO_DEG;
      double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
      double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
      double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

      double gyroXrate = gyroX / 131.0; // Convert to deg/s
      double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
      } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
      } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * dt;
      //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
      //gyroYangle += kalmanY.getRate() * dt;

      compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
      compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

      // Reset the gyro angle when it has drifted too much
      if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
      if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
      File file = SD.open("/Data.txt", FILE_APPEND);
      if (gps.location.isValid())
      {
        if (gps.time.hour() < 10)file.print(F("0"));
        file.print(gps.time.hour());
        file.print("/");
        if (gps.time.minute() < 10) file.print(F("0"));
        file.print(gps.time.minute());
        file.print("/");
        if (gps.time.second() < 10) file.print(F("0"));
        file.print(gps.time.second());
        file.print("/");
        file.print(gps.location.lat(), 6);
        file.print("/");
        file.print(gps.location.lng(), 6);
        file.print("/");
        file.print(gps.location.age());
        file.print("/");
        file.print(roll); file.print("/");
        file.print(gyroXangle); file.print("/");
        file.print(compAngleX); file.print("/");
        file.print(kalAngleX); file.print("/");

        file.print(pitch); file.print("/");
        file.print(gyroYangle); file.print("/");
        file.print(compAngleY); file.print("/");
        file.print(kalAngleY); file.print("/");

        file.print(gps.speed.kmph()); file.print("/");
        file.print(gps.altitude.meters()); file.print("/");
        file.print(gps.satellites.value()); file.print("/");

        double temperature = (double)tempRaw / 340.0 + 36.53;
        file.println(temperature);
        Serial.println(x++);
        delay(250);
      }
    }
    /*
      else
      {
      Serial.println("xx:xx:xx");
      Serial.print(roll); Serial.print("\t");
      Serial.print(gyroXangle); Serial.print("\t");
      Serial.print(compAngleX); Serial.print("\t");
      Serial.print(kalAngleX); Serial.println("\t");

      Serial.print(pitch); Serial.print("\t");
      Serial.print(gyroYangle); Serial.print("\t");
      Serial.print(compAngleY); Serial.print("\t");
      Serial.print(kalAngleY); Serial.println("\t");

      Serial.print(gps.speed.kmph()); Serial.print("\t");
      Serial.print(gps.altitude.meters()); Serial.print("\t");
      Serial.print(gps.satellites.value()); Serial.print("\t");

      double temperature = (double)tempRaw / 340.0 + 36.53;
      Serial.println(temperature);

      }
    */

  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }

}
