
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML6070.h>
#include <MAX44009.h>

#define SEALEVELPRESSURE_HPA (1013.25)

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the  I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// กำหนดขา 8 เป็น RX และขา 9 เป็น TX
static const int RXPin = 16, TXPin = 17;
// กำหนดค่า Baud Rate ของโมดูล GPS = 9600 (ค่า Default)
static const uint32_t GPSBaud = 9600;

TinyGPSPlus myGPS;
SoftwareSerial mySerial(RXPin, TXPin);
Adafruit_BME280 bme;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
MAX44009 light;


#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 490E6

int counter = 0;

void setup() {
  // เริ่ม Serial สำหรับใช้งาน Serial Monitor
  Serial.begin(115200);
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  // เริ่มใช้งาน Software Serial
  mySerial.begin(GPSBaud);

  Serial.println("GPS Module Tutorial");
  Serial.println("This tutorial base on NEO-6M device");
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  uv.begin(VEML6070_1_T);
  if (light.begin())
  {
    Serial.println("Could not find a valid MAX44009 sensor, check wiring!");
    while (1);
  }
}
void loop()
{
  // ถ้า mySerial มีการสื่อสารข้อมูล ให้ library ถอดรหัสข้อมูลแล้วเรียกใช้ฟังก์ชั่น GPSinfo


  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  while (mySerial.available() > 0)
    if (myGPS.encode(mySerial.read()))
      GPSinfo();

  // ถ้ารอ 5 วินาทีแล้วยังไม่มีข้อมูล ให้แสดงข้อความผิดพลาด
  if (millis() > 5000 && myGPS.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    while (true);
  }


}

/*
   ฟังก์ชั่น GPSinfo
*/
void GPSinfo() {
  LoRa.beginPacket();
  LoRa.print(counter);
  LoRa.print("/");
  LoRa.print(myGPS.location.lat());
  LoRa.print("/");
  LoRa.print(myGPS.location.lng());
  LoRa.print("/");
  LoRa.print(bme.readTemperature());
  LoRa.print("/");
  LoRa.print(bme.readPressure() / 100.0F);
  LoRa.print("/");
  LoRa.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  LoRa.print("/");
  LoRa.print(bme.readHumidity());
  LoRa.print("/");
  LoRa.print(accelerometer_x);
  LoRa.print("/");
  LoRa.print(accelerometer_y);
  LoRa.print("/");
  LoRa.print(accelerometer_z);
  LoRa.print("/");
  LoRa.print(gyro_x);
  LoRa.print("/");
  LoRa.print(gyro_y);
  LoRa.print("/");
  LoRa.print(gyro_z);
  LoRa.print("/");
  LoRa.print(uv.readUV());
  LoRa.print("/");
  LoRa.println(light.get_lux());

  LoRa.endPacket();

  counter++;
  delay(400);
}
