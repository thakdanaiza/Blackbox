#include <Adafruit_CCS811.h>
Adafruit_CCS811 ccs;
void setup() {
  Serial.begin(9600);
  Serial.println("CCS811 test");
  if (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    while (1);
  }
}
void loop() {
  if (!ccs.readData()) {
    Serial.print("CO2: ");
    Serial.print(ccs.geteCO2());
    Serial.print("/");
    Serial.print("ppm, TVOC: ");
    Serial.println(ccs.getTVOC());
  }

  delay(800);

}
