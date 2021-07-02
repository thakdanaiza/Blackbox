/*
GY87_Readerv0.ino
This sketch is based on various sketches and libraries by Jeff Rowberg
<jeff@rowberg.net> and discussion thread by pistolero992000 on the Arduino forum
https://forum.arduino.cc/index.php?topic=223345.0.  The BMP180 pressure
Senor library is from Love Electronics Ltd (loveelectronics.com)
http://embedded-lab.com/blog/bmp180/bmp180_11/

It gets all sensors on a GY87 IMU board reporting (something) to the
Serial.  The challenge is the Digital Compass which is nromally blocked by
the Accelerometer.  This is remedied by using an I2C bypass command.  Thanks to
pistolero992000 for this solution.

Sensors on board the GY87 are:
MPU6050 Accelerometer.  Address is 0x68
HMC5883L Digital Compass.  Address is 0x1E
BMP180 Barometer and Temperature Sensor.  Address is 0x77

Connections are through the i2c bus.

SCL to Arduino Pin A5
SDA to Arduino Pin A4
GND, 5V and 3.3V connections also present.

All sensors report sensible values.  The tab seperated data covers a lot of screen space
so you will need to stretch your Serial monitor window to accommodate it.

*/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <BMP180.h>  //Library for the BMP180 barometer.

//MPU6050 Accelerometer 
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//HMC5883L Digital Compass
const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

//The BMP180 Digital Barometer
BMP180 barometer;
// Store the current sea level pressure at your location in Pascals.
float seaLevelPressure = 101325;

int LEDPin = 13;
bool blinkState = false;

int x,y,z; //triple axis data from HMC5883L.

void setup() {
    Wire.begin();
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setI2CBypassEnabled(true);  //This sets the bypass so the HMC5883L gets a look in.

      
    //Initialise the Digital Compass
    Wire.beginTransmission(hmc5883Address);  //Begin communication with compass
    Wire.write(hmc5883ModeRegister);  //select the mode register
    Wire.write(hmcContinuousMode); //continuous measurement mode
    Wire.endTransmission();
    
    //Initialise the BMP180 Barometer (and Temperature Sensor)
    barometer = BMP180();
    // We check to see if we can connect to the BMP180 sensor.
    if(barometer.EnsureConnected())
    {
      Serial.println("Connected to BMP180.");
       // When we have connected, we reset the device to ensure a clean start.
      barometer.SoftReset();
      // Now we initialize the sensor and pull the calibration data.
      barometer.Initialize();
    }
    else
    { 
      Serial.println("No BMP180 sensor found.");
    }
    
    // configure Arduino LED for
    pinMode(LEDPin, OUTPUT);
    delay(10000);
}

void loop() {
    
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LEDPin, blinkState);
    
    //Accessing the HMC5883L Digital Compass  
    //Tell the HMC5883L where to begin reading the data
    Wire.beginTransmission(hmc5883Address);
    Wire.write(hmcDataOutputXMSBAddress);  //Select register 3, X MSB register
    Wire.endTransmission();
  
    //Read data from each axis of the Digital Compass
    Wire.requestFrom(hmc5883Address,6);
    if(6<=Wire.available())
    {
      x = Wire.read()<<8; //X msb
      x |= Wire.read();   //X lsb
      z = Wire.read()<<8; //Z msb
      z |= Wire.read();   //Z lsb
      y = Wire.read()<<8; //Y msb
      y |= Wire.read();   //Y lsb    
    }
  
    int angle = atan2(-y,x)/M_PI*180;
    if (angle < 0)
    {
      angle = angle + 360;
    }
  
    //Reporting the Compass data to the Serial port
    //Serial.print("Compass XYZ:\t");
    //Serial.print(x,y,z);Serial.print("\t");
    Serial.print("Dir(deg):\t");
    Serial.print(angle); Serial.print("\t");
    
    if(barometer.IsConnected)
    {
      long currentPressure = barometer.GetPressure();
      
      // Print out the Pressure.
      Serial.print("BMP180 P:\t");
      Serial.print(currentPressure);Serial.print("Pa");Serial.print("\t");
            
      // Retrieve the current altitude (in meters). Current Sea Level Pressure is required for this.
      float altitude = barometer.GetAltitude(seaLevelPressure);
    
      // Print out the Altitude.
      Serial.print("Alt:\t");
      Serial.print(altitude);Serial.print(" m");Serial.print("\t");
    
      // Retrieve the current temperature in degrees celcius.
      float currentTemperature = barometer.GetTemperature();
    
      // Print out the Temperature
      Serial.print("Temp:\t");
      Serial.print(currentTemperature);Serial.println("C");
    }   
}

