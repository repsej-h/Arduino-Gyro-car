#include <EEPROM.h>

// libs voor gyrosensor:
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


struct sensorReading {
  long millis;
  int value;
  uint8_t speed;
};
// voeding arduino, via GND en Vin rechtstreeks aan GND en VSS van motor module
// niet via 5V, die levert niet genoeg spanning

// Hieronder de definities van de pinnen, met kleurcodering. Dit zou je normaal gezien niet moeten aanpassen
#pragma region Pindefinitions
// Motor FRONT connections
const uint8_t enB_f = 3; // blauw
const uint8_t enA_f = 9; // bruin
const uint8_t in4_f = 4; // groen
const uint8_t in3_f = 5; // geel
const uint8_t in2_f = 7; // oranje
const uint8_t in1_f = 8; // rood

// Motor BACK connections
const uint8_t enB_b = 6; // blauw
const uint8_t enA_b = 10; // bruin
const uint8_t in4_b = 13; // groen
const uint8_t in3_b = 12; // geel
const uint8_t in2_b = 11; // oranje
const uint8_t in1_b = 2; // rood

const uint8_t sensorPin = A0; // Analoge pin voor de sensor
const uint8_t eepromSize = 2; // Grootte van de EEPROM op de Arduino Uno

// pins voor gyro / accelerometer
const uint8_t sda = A4;
const uint8_t scl = A5;

#pragma endregion Pindefinitions

// Hieronder de PWM waarden voor de trage en snelle snelheden, deze mag je niet aanpassen enkel gebruiken
#pragma region Drivingspeeds
const uint8_t highSpeed = 255;
const uint8_t lowSpeed = 127;
uint8_t currentSpeed = 0;
#pragma endregion Drivingspeeds

// hieronder de snelheid waarmee gedraaid word en de tijd nodig om te draaien. Deze tijd kan afhankelijk van de ondergrond en het wagentje moeten worden aangepast zodat een mooie 90 graden gedraaid wordt
const uint8_t turningSpeed = 127;
const long turningTime = 715;

// Deze waarde bevat de totale sensorwaarde van de sensor aangesloten op de sensorpin
const int maxNumberOfReadings = 200;
sensorReading sensorValues[maxNumberOfReadings];
int readingIndex = 0;

// voor funtionering gyro sensor
Adafruit_MPU6050 mpu;

long previousTime = 0;
float elapsedTime;

float yaw = 0; // Angle around the Z-axis (yaw)

void setup() {
  
  // For Debugging purposes
  Serial.begin(115200);
  while(!Serial);
  // EEPROM leegmaken
  // for (int i = 0; i < eepromSize; i++) {
  //   EEPROM.write(i, 0); // Schrijf 0 naar elk adres
  // }
  // Set all the motor control pins to outputs
	pinMode(enA_f, OUTPUT);
	pinMode(enB_f, OUTPUT);
	pinMode(in1_f, OUTPUT);
	pinMode(in2_f, OUTPUT);
	pinMode(in3_f, OUTPUT);
	pinMode(in4_f, OUTPUT);

  pinMode(enA_b, OUTPUT);
  pinMode(enB_b, OUTPUT);
  pinMode(in1_b, OUTPUT);
  pinMode(in2_b, OUTPUT);
  pinMode(in3_b, OUTPUT);
  pinMode(in4_b, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1_f, LOW);
	digitalWrite(in2_f, LOW);
	digitalWrite(in3_f, LOW);
	digitalWrite(in4_f, LOW);
  digitalWrite(in1_b, LOW);
  digitalWrite(in2_b, LOW);
  digitalWrite(in3_b, LOW);
  digitalWrite(in4_b, LOW);
  
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    exit(1);
  } 
  Serial.println("MPU6050 Found!");

  mpu.setGyroRange(0);

  previousTime = micros();

}

void loop() {
  readGyro();
  saveSensorValueToEEPROM();
      
}


#pragma region Helperfunctions
/**
* Rijd rechtdoor gedurende [time] milliseconden
*/
void drive(uint8_t speed, int time){
    setSpeed(speed);
    // Turn on motor A & B
    digitalWrite(in1_f, HIGH);
    digitalWrite(in2_f, LOW);
    digitalWrite(in3_f, LOW);
    digitalWrite(in4_f, HIGH);

    digitalWrite(in1_b, LOW);
    digitalWrite(in2_b, HIGH);
    digitalWrite(in3_b, HIGH);
    digitalWrite(in4_b, LOW);
    
    readSensorAndWait(time);

    digitalWrite(in1_f, LOW);
    digitalWrite(in2_f, LOW);
    digitalWrite(in3_f, LOW);
    digitalWrite(in4_f, LOW);
    digitalWrite(in1_b, LOW);
    digitalWrite(in2_b, LOW);
    digitalWrite(in3_b, LOW);
    digitalWrite(in4_b, LOW);
}

/**
* Draai 90 graden naar links
*/
void left(){
  setSpeed(turningSpeed);

  digitalWrite(in1_f, LOW);
  digitalWrite(in2_f, HIGH);
  digitalWrite(in3_f, LOW);
  digitalWrite(in4_f, HIGH);

  digitalWrite(in1_b, LOW);
  digitalWrite(in2_b, HIGH);
  digitalWrite(in3_b, LOW);
  digitalWrite(in4_b, HIGH);

  delay(turningTime);

  digitalWrite(in1_f, LOW);
  digitalWrite(in2_f, LOW);
  digitalWrite(in3_f, LOW);
  digitalWrite(in4_f, LOW);

  digitalWrite(in1_b, LOW);
  digitalWrite(in2_b, LOW);
  digitalWrite(in3_b, LOW);
  digitalWrite(in4_b, LOW);
}

/**
* Draai 90 graden naar rechts
*/
void right(){
  setSpeed(turningSpeed);

  digitalWrite(in1_f, HIGH);
  digitalWrite(in2_f, LOW);
  digitalWrite(in3_f, HIGH);
  digitalWrite(in4_f, LOW);

  digitalWrite(in1_b, HIGH);
  digitalWrite(in2_b, LOW);
  digitalWrite(in3_b, HIGH);
  digitalWrite(in4_b, LOW);

  delay(turningTime);


  digitalWrite(in1_f, LOW);
  digitalWrite(in2_f, LOW);
  digitalWrite(in3_f, LOW);
  digitalWrite(in4_f, LOW);

  digitalWrite(in1_b, LOW);
  digitalWrite(in2_b, LOW);
  digitalWrite(in3_b, LOW);
  digitalWrite(in4_b, LOW);
}

void setSpeed(uint8_t speed){
  analogWrite(enA_f, speed);
  analogWrite(enB_f, speed);
  analogWrite(enA_b, speed);
  analogWrite(enB_b, speed);
  currentSpeed = speed;
}

/**
* Voert een delay uit terwijl de sensorwaarden blijven uitgelezen worden.
*/
void readSensorAndWait(unsigned long waitTime){
  byte numSamples = 5;
  unsigned long ms = waitTime/(numSamples -1);
  unsigned long correction = waitTime - (numSamples-1)*ms;
  for(byte i = 0; i < numSamples-1; i++){
    readSensor();
    delay(ms);
  }
  delay(correction);
  readSensor();
}

void readSensor(){
    if(readingIndex >= maxNumberOfReadings) return;
    sensorReading s;
    s.millis = millis();
    s.value = analogRead(sensorPin);
    s.speed = currentSpeed;
    sensorValues[readingIndex] = s;
    readingIndex++;
}

void readGyro(){
  // Get new sensor events (accelerometer and gyroscope)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate elapsed time
  long currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0; // seconds
  previousTime = currentTime;

  // Gyroscope readings are in rad/s (radians per second)
  // The 'g' event contains gyroscope data: g.gyro.x, g.gyro.y, g.gyro.z
  // We are interested in rotation around the Z-axis, which is g.gyro.z
  
  float yawRate = g.gyro.z * 180 / PI; // Convert radians/second to degrees/second

  // Integrate the yaw rate to get the yaw angle
  yaw += yawRate * elapsedTime;

  /* for debugging

  Serial.print("Yaw Angle (Z-axis): ");
  Serial.print(yaw);
  Serial.println(" degrees");

  */
  delay(10);
}

void saveSensorValueToEEPROM(){
    EEPROM.write(0,readingIndex);
    for(byte i = 0; i < readingIndex; i++){
      long millis = sensorValues[i].millis;
      int sensorValue = sensorValues[i].value;
      int speed = sensorValues[i].speed;
      if(sensorValue == 0) break;
      Serial.print(millis);
      Serial.print(";");
      Serial.print(sensorValue);
      Serial.println();
  	  byte adress = (7*i)+1;

      EEPROM.write(adress, (millis >> 24) & 0xFF); // Opslaan (8-bit max, schaal naar 0-255)
      EEPROM.write(adress+1, (millis >> 16) & 0xFF); // Opslaan (8-bit max, schaal naar 0-255)
      EEPROM.write(adress+2, (millis >> 8) & 0xFF); // Opslaan (8-bit max, schaal naar 0-255)
      EEPROM.write(adress+3, millis & 0xFF); // Opslaan (8-bit max, schaal naar 0-255)
      EEPROM.write(adress+4, (sensorValue >> 8) & 0xFF ); // voorste 8 bits
      EEPROM.write(adress+5, sensorValue & 0xFF); // achterste 8 bits 0xFF = 255
      EEPROM.write(adress+6, speed); // achterste 8 bits 0xFF = 255
    }
}
#pragma endregion Helperfunctions
