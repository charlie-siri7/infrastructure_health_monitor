#include "DHT.h"

#include <ESP32Servo.h>

#define DHTPIN 14  // Set the pin connected to the DHT11 data pin
#define DHTTYPE DHT11 // DHT 11 

DHT dht(DHTPIN, DHTTYPE);
int counter;
int ledCounter;

int analogValue;
float humidity;
float temperature;
int potValue;
uint32_t voltage_mV;

int alerts[2] = {0, 0};
int ventilation = false;
int angle = 0;

int target = 0;

// Define the servo and the pin it is connected to
Servo myServo;
const int servoPin = 25;

// Define the minimum and maximum pulse widths for the servo
const int minPulseWidth = 500; // 0.5 ms
const int maxPulseWidth = 2500; // 2.5 ms

const int potPin = 34; // Potentiometer connected to

const int redLedPin = 27;  // The GPIO pin for the red LED
const int yellowLedPin = 26;  // The GPIO pin for the yellow LED
const int greenLedPin = 33;  // The GPIO pin for the green LED
const int blueLedPin = 32;  // The GPIO pin for the green LED

// PWM settings
const int freq = 5000; // PWM frequency
const int resolution = 12; // PWM resolution (bits)

const int delayTime = 20; // 20 ms delay

void setup() {
  // Configure PWM
  // ledcAttach(ledPin, freq, resolution);

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
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
}

void loop() {
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  myServo.writeMicroseconds(pulseWidth);
  Serial.printf("angle = %d, ", angle);
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
    if (potValue < 2500) {
      alerts[0] = 0;
    } else if (potValue <= 3500) {
      alerts[0] = 1;
    } else {
      alerts[0] = 2;
    }
  }

  // Get moisture sensor value
  if (counter % (300 / delayTime) == 0) {
    // Read the analog value
    analogValue = analogRead(35);

    // Update alerts based on moisture sensor value
    if (analogValue > 3000) {
      alerts[1] = 0;
    } else if (analogValue >= 1500) {
      alerts[1] = 1;
    } else {
      alerts[1] = 2;
    }
  }

  // Wait a few seconds between measurements.
  if (counter % (2000 / delayTime) == 0) {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    if (temperature > 35 || humidity > 80) {
      ventilation = true;
    } else {
      ventilation = false;
    }

    if (ventilation) {
      digitalWrite(blueLedPin, HIGH);
    } else {
      digitalWrite(blueLedPin, LOW);
    }
  }

  if (alerts[0] == 0 || alerts[1] == 0) {
    digitalWrite(greenLedPin, HIGH);
  } else {
    digitalWrite(greenLedPin, LOW);
  }

  if (alerts[0] == 1 || alerts[1] == 1) {
    digitalWrite(yellowLedPin, HIGH);
  } else {
    digitalWrite(yellowLedPin, LOW);
  }

  if (alerts[0] == 2 || alerts[1] == 2) {
    digitalWrite(redLedPin, HIGH);
    target = 90;
  } else {
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
    Serial.print("Potentiometer Value: ");
    Serial.print(potValue);

    Serial.print(", Voltage: ");
    Serial.print(voltage_mV / 1000.0); // Convert millivolts to volts
    Serial.print(" V, ");

    // Print out the values for moisture sensor
    Serial.printf("Analog value = %d, ", analogValue);

    // Print the humidity and temperature
    Serial.print("Humidity: "); 
    Serial.print(humidity);
    Serial.print(", ");
    Serial.print("Temperature: "); 
    Serial.print(temperature);
    Serial.println(" *C");
  }
}