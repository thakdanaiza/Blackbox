/*
   Connect the SD card to the following pins:

   SD Card | ESP32
      D2       -
      D3       SS
      CMD      MOSI
      VSS      GND
      VDD      3.3V
      CLK      SCK
      VSS      GND
      D0       MISO
      D1       -
*/
#include "FS.h"
#include "SD.h"
#include "SPI.h"


void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void setup() {
  Serial.begin(115200);
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  //writeFile(SD, "/Data.txt", "");
  readFile(SD, "/Data.txt");

}

void loop() {

}
