#include "DHT.h"

#include <ESP32Servo.h>

#define DHTPIN 14  // Set the pin connected to the DHT11 data pin
#define DHTTYPE DHT11 // DHT 11 

DHT dht(DHTPIN, DHTTYPE);
int counter;
int ledCounter;

int moistureValue;
float humidity;
float temperature;
int potValue;
uint32_t voltage_mV;

int alerts[3] = {0, 0, 0};
int ventilation = false;
int angle = 0;

int target = 0;

String warningString;
String alertString;

// Define the servo and the pin it is connected to
Servo myServo;
const int servoPin = 25;

// Define the minimum and maximum pulse widths for the servo
const int minPulseWidth = 500; // 0.5 ms
const int maxPulseWidth = 2500; // 2.5 ms

const int potPin = 34; // Potentiometer connected to

const int redLedPin = 27;  // The GPIO pin for the red LED
const int yellowLedPin = 26;  // The GPIO pin for the yellow LED

// PWM settings
const int freq = 5000; // PWM frequency
const int resolution = 12; // PWM resolution (bits)

const int delayTime = 20; // 20 ms delay

// < 2500 is safe, < 3500 is warning, otherwise alert
const int pot_safe_warning_threshold = 2500;
const int pot_warning_alert_threshold = 3500;

// > 3000 is safe, > 1500 is warning, otherwise alert
const int moisture_sensor_safe_warning_threshold = 2500; // 3000
const int moisture_sensor_warning_alert_threshold = 1500;

const int max_temp = 35; 
const int max_humidity = 45; // Make max_humidity 80 after testing

void setup() {
  // Attach the servo to the specified pin and set its pulse width range
  myServo.attach(servoPin, minPulseWidth, maxPulseWidth);

  // Set the PWM frequency for the servo
  myServo.setPeriodHertz(50); // Standard 50Hz servo

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println("DHT11 test!");
  dht.begin();
  counter = 0;
  ledCounter = 0;

  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
}

void loop() {
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  myServo.writeMicroseconds(pulseWidth);
  if (angle < target) {
    angle++; 
  }
  if (angle > target) {
    angle--; 
  }

  updateValues();
  printValues();
  delay(delayTime);
  counter++;
}

void updateValues() {
  // Get potentiometer value
  if (counter % (100 / delayTime) == 0) {
    potValue = analogRead(potPin); // read the value of the potentiometer
    voltage_mV = analogReadMilliVolts(potPin); // Read the voltage in millivolts
    // ledcWrite(ledPin, potValue);

    // Update alerts based on potentiometer value
    if (potValue < pot_safe_warning_threshold) {
      alerts[0] = 0;
    } else if (potValue <= pot_warning_alert_threshold) {
      alerts[0] = 1;
    } else {
      alerts[0] = 2;
    }
  }

  // Get moisture sensor value
  if (counter % (300 / delayTime) == 0) {
    // Read the analog value
    moistureValue = analogRead(35);

    // Update alerts based on moisture sensor value
    if (moistureValue > moisture_sensor_safe_warning_threshold) {
      alerts[1] = 0;
    } else if (moistureValue >= moisture_sensor_warning_alert_threshold) {
      alerts[1] = 1;
    } else {
      alerts[1] = 2;
    }
  }

  // Wait a few seconds between measurements.
  if (counter % (2000 / delayTime) == 0) {
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    if (temperature > max_temp || humidity > max_humidity) {
      alerts[2] = 1;
    } else {
      alerts[2] = 0;
    }
  }

  if (alerts[0] == 1 || alerts[1] == 1 || alerts[2] == 1) {
    warningString = "Warnings: ";
    if (alerts[0] == 1) {
      warningString += String("Potentiometer value is greater than ") + String(pot_safe_warning_threshold) + " (" + String(potValue) + ").\n";  
    }
    if (alerts[1] == 1) {
      warningString += String("Moisture sensor value is less than ") + String(moisture_sensor_safe_warning_threshold) + " (" + String(moistureValue) + ")" + ".\n";
    }
    if (alerts[2] == 1) {
      if (temperature > max_temp) {
        warningString += String("Temperature is above ") + String(max_temp) + " (" + String(temperature) + ")" + ".\n";
      } 
      if (humidity > max_humidity) {
        warningString += String("Humidity is above ") + String(max_humidity) + " (" + String(humidity) + ")" + ".\n";
      }
    }
    digitalWrite(yellowLedPin, HIGH);
  } else {
    warningString = "Warnings: none.\n";
    digitalWrite(yellowLedPin, LOW);
  }

  if (alerts[0] == 2 || alerts[1] == 2) {
    alertString = "Alerts: ";
    if (alerts[0] == 2) {
      alertString += String("Potentiometer value is greater than ") + String(pot_warning_alert_threshold) + " (" + String(potValue) + ")" + ".\n";
    }
    if (alerts[1] == 2) {
      alertString += String("Moisture sensor value is less than ") + String(moisture_sensor_warning_alert_threshold) + " (" + String(moistureValue) + ")" + ".\n";
    }
    digitalWrite(redLedPin, HIGH);
    target = 90;
  } else {
    alertString = "Alerts: none.\n";
    digitalWrite(redLedPin, LOW);
    if (ventilation) {
      target = 45;
    } else {
      target = 0;
    }
  }

}

void printValues() {
  if (counter % (100 / delayTime) == 0) {
    Serial.print(warningString);
    Serial.println(alertString);
    Serial.printf("angle = %d, ", angle);
    Serial.print("Potentiometer Value: ");
    Serial.print(potValue);

    Serial.print(", Voltage: ");
    Serial.print(voltage_mV / 1000.0); // Convert millivolts to volts
    Serial.print(" V, ");

    // Print out the values for moisture sensor
    Serial.printf("Moisture value = %d, ", moistureValue);

    // Print the humidity and temperature
    Serial.print("Humidity: "); 
    Serial.print(humidity);
    Serial.print(", ");
    Serial.print("Temperature: "); 
    Serial.print(temperature);
    Serial.println(" *C");

     Serial.println("");

  }
}