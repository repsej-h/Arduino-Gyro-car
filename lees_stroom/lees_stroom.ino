#include <EEPROM.h>
// voeding arduino, via GND en Vin rechtstreeks aan GND en VSS van motor module
// niet via 5V, die levert niet genoeg spanning

void setup() {
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB port only
}

void loop() {
  Serial.println();
  int numberOfReadings = EEPROM.read(0);
  Serial.print("tijd(ms);stroom(A);spanning(V)");
  for(int i = 0; i < numberOfReadings; i++){
      byte b1, b2, b3, b4, b5, b6, speed;
      byte adress = (7*i)+1;
      b1 = EEPROM.read(adress); // Opslaan (8-bit max, schaal naar 0-255)
      b2 = EEPROM.read(adress+1); // Opslaan (8-bit max, schaal naar 0-255)
      b3 = EEPROM.read(adress+2); // Opslaan (8-bit max, schaal naar 0-255)
      b4 = EEPROM.read(adress+3); // Opslaan (8-bit max, schaal naar 0-255)
      b5 = EEPROM.read(adress+4); // voorste 8 bits
      b6 = EEPROM.read(adress+5); // achterste 8 bits 0xFF = 255
      speed = EEPROM.read(adress+6); // achterste 8 bits 0xFF = 255

      long millis = b4 | b3 << 8 | b2 << 16 | b1 << 24;
      int sensorValue = b6 | b5 << 8;

      float current = (526-sensorValue)*0.0488;
      float voltage = (10.0*speed)/255;
      Serial.println();
      Serial.print(millis);
      Serial.print(";");
      Serial.print(current);
      Serial.print(";");
      Serial.print(voltage);
    }
    for(int i = 0; i< 10; i++){
      Serial.println();
    }
    while(true);
}
