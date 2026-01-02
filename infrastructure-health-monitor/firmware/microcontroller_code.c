#include "DHT.h"
#include <ESP32Servo.h>

#define DHTPIN 14
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
Servo myServo;

// Constants
const int servoPin = 25;
const int potPin = 34;
const int moisturePin = 35;
const int redLedPin = 27;
const int yellowLedPin = 26;
const int minPulseWidth = 500;
const int maxPulseWidth = 2500;
const int delayTime = 20;

// Thresholds
const int pot_safe_warning_threshold = 2500;
const int pot_warning_alert_threshold = 3500;
const int moisture_safe_threshold = 2500;
const int moisture_alert_threshold = 1500;
const int max_temp = 35;

// Variables
int angle = 0;
int target = 0;
int counter = 0;
float humidity, temperature;
int potValue, moistureValue;
int alerts[3] = {0, 0, 0}; // 0: Pot, 1: Moisture, 2: Temp/Hum

void setup() {
  Serial.begin(115200);
  dht.begin();
  
  myServo.attach(servoPin, minPulseWidth, maxPulseWidth);
  myServo.setPeriodHertz(50);
  
  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  
  // No more blocking while() loops for input. 
  // The app will send configuration commands if needed.
}

void loop() {
  // Servo Movement Logic
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  myServo.writeMicroseconds(pulseWidth);
  
  if (angle < target) angle++;
  if (angle > target) angle--;

  updateSensors();
  handleLogic();

  // Send JSON data to Python every 500ms
  if (counter % (500 / delayTime) == 0) {
    sendJsonTelemetry();
  }

  // Check for incoming commands from the App
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("SET_ANGLE:")) {
      target = cmd.substring(10).toInt();
    }
  }

  delay(delayTime);
  counter++;
}

void updateSensors() {
  potValue = analogRead(potPin);
  moistureValue = analogRead(moisturePin);
  
  if (counter % (2000 / delayTime) == 0) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
  }
}

void handleLogic() {
  // Potentiometer Alert Logic
  if (potValue < pot_safe_warning_threshold) alerts[0] = 0;
  else if (potValue <= pot_warning_alert_threshold) alerts[0] = 1;
  else alerts[0] = 2;

  // Moisture Alert Logic
  if (moistureValue > moisture_safe_threshold) alerts[1] = 0;
  else if (moistureValue >= moisture_alert_threshold) alerts[1] = 1;
  else alerts[1] = 2;

  // Visual/Hardware Output
  bool isWarning = (alerts[0] == 1 || alerts[1] == 1 || temperature > max_temp);
  bool isAlert = (alerts[0] == 2 || alerts[1] == 2);

  digitalWrite(yellowLedPin, isWarning ? HIGH : LOW);
  digitalWrite(redLedPin, isAlert ? HIGH : LOW);

  if (isAlert) target = 90; // Emergency Shut-off
}

void sendJsonTelemetry() {
  // Sending a flat JSON string for the Python backend to parse
  Serial.print("{");
  Serial.print("\"temp\":" + String(temperature) + ",");
  Serial.print("\"hum\":" + String(humidity) + ",");
  Serial.print("\"moisture\":" + String(moistureValue) + ",");
  Serial.print("\"pot\":" + String(potValue) + ",");
  Serial.print("\"angle\":" + String(angle) + ",");
  Serial.print("\"alert_level\":" + String(alerts[0] + alerts[1])); 
  Serial.println("}");
}