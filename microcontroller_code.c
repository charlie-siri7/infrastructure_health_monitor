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
  // Rotate the servo from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
    myServo.writeMicroseconds(pulseWidth);
    updateValues();
    printValues();
    delay(delayTime);
    counter++;
  }

  // Rotate the servo from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle--) {
    int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
    myServo.writeMicroseconds(pulseWidth);
    updateValues();
    printValues();
    delay(delayTime);
    counter++;
  }
}

void updateValues() {
    if (counter % (100 / delayTime) == 0) {
    potValue = analogRead(potPin); // read the value of the potentiometer
    voltage_mV = analogReadMilliVolts(potPin); // Read the voltage in millivolts
    // ledcWrite(ledPin, potValue);
  }


  if (counter % (300 / delayTime) == 0) {
    // Read the analog value
    analogValue = analogRead(35);
  }

  // Wait a few seconds between measurements.
  if (counter % (1000 / delayTime) == 0) {
    if (ledCounter % 4 == 0) {
      digitalWrite(redLedPin, HIGH);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(blueLedPin, LOW);
    } else if (ledCounter % 4 == 1) {
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(blueLedPin, LOW);
    } else if (ledCounter % 4 == 2) {
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(blueLedPin, LOW);
    } else {
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(blueLedPin, HIGH);
    }
    ledCounter++;
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
  }
}

void printValues() {
  if (counter % (100 / delayTime) == 0) {
    Serial.print("Potentiometer Value: ");
    Serial.print(potValue);

    Serial.print(", Voltage: ");
    Serial.print(voltage_mV / 1000.0); // Convert millivolts to volts
    Serial.print(" V, ");

    // Print out the values
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