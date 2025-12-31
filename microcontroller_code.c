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

int alertDelay = 0;
int statisticDelay = 0;

String warningString;
String alertString;

// Define the servo and the pin it is connected to
Servo myServo;

String metricString;

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
  delay(3000);

  // Clear previous lines
  for (int i = 0; i < 100; i++) {
    Serial.println(); 
  }
  Serial.println("DHT11 test!");
  dht.begin();
  counter = 0;
  ledCounter = 0;

  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);

  while (alertDelay < 100) {
    Serial.println("Enter an integer representing how many milliseconds you would like between alert displays (must be >= 100): ");
    // int input = Serial.read();

    while (Serial.available() == 0) {
      // Wait for user input
    }

    alertDelay = Serial.parseInt();

    // Clear buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    if (alertDelay < 100) {
      Serial.print("Invalid input: ");
      Serial.print(alertDelay);
      Serial.println(". The value must be at least 100.");
    }

  }

  Serial.print("Alert delay set to: ");
  Serial.println(alertDelay);

  // Clear buffer
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.print("Would you also like to display at least 1 other metric of the system\n(ex: arm angle, potentiometer value, voltage, moisture value, humidity, temperature)?: ");
  
  while (Serial.available() == 0) {
    // Wait for user input
  }

  String yesOrNo = Serial.readStringUntil('\n');
  Serial.print("Response: ");
  Serial.println(yesOrNo);

  if ((yesOrNo.indexOf("y") != -1) || (yesOrNo.indexOf("Y") != -1)) {
    // Clear buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.print("\nEnter a number containing all the digits corresponding to what metrics you want to see: \n1 = arm angle \n2 = potentiometer value \n3 = voltage \n4 = moisture value \n5 = humidity \n6 = temperature \n(ex: enter '123456' to see all metrics)");

    while (Serial.available() == 0) {
      // Wait for user input
    }

    metricString = Serial.readStringUntil('\n');

    Serial.print(metricString);

    // Clear buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    while (statisticDelay < 100) {
      Serial.println("\nEnter an integer representing how many milliseconds you would like between statistic displays (must be >= 100): ");
      // int input = Serial.read();

      while (Serial.available() == 0) {
        // Wait for user input
      }

      statisticDelay = Serial.parseInt();

      // Clear buffer
      while (Serial.available() > 0) {
        Serial.read();
      }

      if (statisticDelay < 100) {
        Serial.print("Invalid input: ");
        Serial.print(statisticDelay);
        Serial.println(". The value must be at least 100.");
      }

    }

    Serial.print("Statistic delay set to: ");
    Serial.println(statisticDelay);
  }
  
  delay(1000);
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
  if (counter % (alertDelay / delayTime) == 0) {
    Serial.print(warningString);
    Serial.println(alertString);

    // Serial.println("");

  }

  if (counter % (statisticDelay / delayTime) == 0) {
    // 1 = arm angle 
    // 2 = potentiometer value 
    // 3 = voltage 
    // 4 = moisture value 
    // 5 = humidity 
    // 6 = temperature \n(ex: enter '123456' to see all metrics)");

    if (metricString.indexOf("1") != -1) {
      Serial.printf("angle = %d, ", angle);
    }
    
    if (metricString.indexOf("2") != -1) {
      Serial.printf("Potentiometer Value: %d, ", potValue);
      // Serial.print(potValue);
    }

    if (metricString.indexOf("3") != -1) {
      Serial.print(", Voltage: ");
      Serial.print(voltage_mV / 1000.0); // Convert millivolts to volts
      Serial.print(" V, ");
    }

    if (metricString.indexOf("4") != -1) {
      // Print out the values for moisture sensor
      Serial.printf("Moisture value = %d, ", moistureValue);
    }

    if (metricString.indexOf("6") != -1) {
      // Print the humidity and temperature
      Serial.print("Humidity: "); 
      Serial.print(humidity);
      Serial.print(", ");
    }

    if (metricString.indexOf("7") != -1) {
      Serial.print("Temperature: "); 
      Serial.print(temperature);
      Serial.println(" *C");
    }

  }
}