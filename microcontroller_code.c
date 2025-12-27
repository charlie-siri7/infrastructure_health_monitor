#include "DHT.h"

#define DHTPIN 14  // Set the pin connected to the DHT11 data pin
#define DHTTYPE DHT11 // DHT 11 

DHT dht(DHTPIN, DHTTYPE);
int counter;

const int potPin = 34; // Potentiometer connected to
const int ledPin = 26; // LED connected to

// PWM settings
const int freq = 5000; // PWM frequency
const int resolution = 12; // PWM resolution (bits)

void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println("DHT11 test!");
  dht.begin();
  counter = 0;

  // Configure PWM
  ledcAttach(ledPin, freq, resolution);
}

void loop() {

  int potValue = analogRead(potPin); // read the value of the potentiometer
  uint32_t voltage_mV = analogReadMilliVolts(potPin); // Read the voltage in millivolts

  ledcWrite(ledPin, potValue);

  Serial.print("Potentiometer Value: ");
  Serial.print(potValue);
  Serial.print(", Voltage: ");
  Serial.print(voltage_mV / 1000.0); // Convert millivolts to volts
  Serial.println(" V");

  if (counter % 3 == 0) {
    // Read the analog value
    int analogValue = analogRead(35);
    
    // Print out the values
    Serial.printf("Analog value = %d\n", analogValue);
  }

  // Wait a few seconds between measurements.
  if (counter % 20 == 0) {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    float humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    // Print the humidity and temperature
    Serial.print("Humidity: "); 
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(temperature);
    Serial.println(" *C");
  }

  delay(100);  // delay between reads for clear read from serial monitor
  counter++;

}