
// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Your GPRS credentials, if any
const char apn[] = "Internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "true";
const char gprsPass[] = "true";

// MQTT details
const char* broker = "134.122.110.253";                    // Public IP address or domain name
const char* mqttUsername = "safeside";  // MQTT username
const char* mqttPassword = "safeside_pass_j9SZFA8EIm";  // MQTT password
const char* topicOutput1 = "ESP32/car/err/0640367407";
const char* reportArray = "ESP32/0640367406/report";
const char* SIMCardName = "";

#include <Wire.h>
#include <TinyGsmClient.h>
#include <TinyGPS++.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

HardwareSerial neogps(0);
TinyGPSPlus gps;
float latitude , longitude;
String  latitude_string , longitiude_string;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
unsigned char startTime;
int rebootTime = 61;

TinyGsmClient client(modem);
PubSubClient mqtt(client);

String payload;

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

#define I2C_SDA              21
#define I2C_SCL              22
#define I2C_SDA_2            18
#define I2C_SCL_2            19

uint32_t lastReconnectAttempt = 0;
uint8_t i2cData[14]; // Buffer for I2C data

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

long lastMsg = 0;

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker without username and password
  boolean status = mqtt.connect("GsmClientN");

  // Or, if you want to authenticate MQTT:
  //boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(topicOutput1);
  SerialMon.print("Subscribe : ");
  SerialMon.println(topicOutput1);
  return mqtt.connected();
}


void setup() {
  // Set console baud rate
  SerialMon.begin(9600);
  delay(10);
  neogps.begin(9600, SERIAL_8N1, 3, 1);
  Serial.println("neogps serial initialize");
  // Start I2C communication
  //I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  //bool isOk = setPowerBoostKeepOn(1);
  //SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);


  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }
  Wire.begin();
  /*
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

   kalman and gyro starting angle 
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
*/
  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    if (mqttConnect()) {
      SerialMon.println("=== MQTT CONNECTED ===");
    }
    delay(100);
  }
}

void loop() {

  while (neogps.available() > 0) {
    //SerialMon.println("8");

    if (gps.encode(neogps.read()))
    {
      if (gps.location.isValid())
      {
        if (gps.date.day() < 10)payload += "0" ;
        payload += gps.date.day() ; payload += ",";
        if (gps.date.month() < 10) payload += "0" ;
        payload += gps.date.month() ; payload += ",";
        if (gps.time.hour() < 10)payload += "0" ;
        payload += gps.time.hour() ; payload += ",";
        if (gps.time.minute() < 10) payload += "0" ;
        payload += gps.time.minute() ; payload += ",";
        if (gps.time.second() < 10) payload += "0" ;
        payload += gps.time.second() ; payload += ",";

        char bufnewlat[10];
        //gcvt(gps.location.lat(), 7, bufnewlat);
        dtostrf(gps.location.lat(), 3, 7, bufnewlat);
        char bufnewlng[10];
        //gcvt(gps.location.lng(), 7, bufnewlng);
        dtostrf(gps.location.lng(), 3, 7, bufnewlng);
        payload += bufnewlat ; payload += ",";
        payload += bufnewlng ; payload += ",";
        /*
        while (i2cRead(0x3B, i2cData, 14));
        accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
        accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
        accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
        tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
        gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
        gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
        gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

        payload += accX ; payload += ",";
        payload += accY ; payload += ",";
        payload += accZ ; payload += ",";

        payload += gyroX ; payload += ",";
        payload += gyroY ; payload += ",";
        payload += gyroZ ; payload += ",";
*/
        payload += gps.speed.kmph() ; payload += ",";
        payload += gps.altitude.meters() ; payload += ",";
        int payload_len = payload.length() + 1;

        // Prepare the character array (the buffer)
        char char_array[payload_len];
        payload.toCharArray(char_array, payload_len);
        SerialMon.println(payload);
        mqtt.publish(topicOutput1, char_array);
        payload = "";
        delay(5000);
        SerialMon.print(mqtt.connect("GsmClientN"));
        if (!mqtt.connected()) {
            SerialMon.println("=== MQTT NOT CONNECTED ===");
            // Reconnect every 10 seconds
            if (mqttConnect()) {
              SerialMon.println("=== MQTT CONNECTED ===");
            }
            delay(100);
          }
        /*if (startTime >= 61) {
          startTime = gps.time.minute();
          rebootTime = startTime + 5;
          if (rebootTime >= 60) {
            rebootTime = rebootTime - 60;
          }
        }
        if (gps.time.minute() >= rebootTime) {
          
          if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.println(" fail");
            ESP.restart();
          }
          else {
            SerialMon.println(" OK");
          }
          mqtt.publish(topicOutput1, reportArray);
          //ESP.restart();
        }*/
      }
    }
  }
  //delay(5000);
}
