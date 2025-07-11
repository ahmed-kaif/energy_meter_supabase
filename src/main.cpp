#include <Arduino.h>
#include <ESPSupabase.h>
#include <WiFi.h>

// Add you Wi-Fi credentials
const char *ssid = "Desh";
const char *password = "12354678";

// Supabase credentials
const char *supabaseUrl = "";
const char *supabaseKey = "";

Supabase supabase;
// Add the table name here
String tableName = "energy";

// Pin definitions
const int currentPin = 39; // ACS712
const int voltagePin = 36; // ZMPT101B
const int ledPin = 2;

// Energy variables
float energy = 0;      // in joules
float totalEnergy = 0; // in joules
unsigned long lastTime;

// Current Sensor Calibration Config
const int ACS712_Pin = 36; // ESP32-S3 Youth pin (A0) connected to ACS712 sensor
const float Sensitivity =
    185.0; // Sensor sensitivity (185 mV/A for the 5A version)
const float ADC_Resolution = 4095.0;  // ESP32 ADC resolution (12-bit)
const float V_Ref = 5.0;              // Reference voltage for the divider (5V)
const float VoltageDividerGain = 1.5; // Voltage divider gain (1k + 2k) / 2k
const float noiseThreshold =
    0.02; // Ignore current below 20 mA (noise reduction)
const float calibrationFactor =
    1.075; // Calibration factor (adjust to match multimeter readings)

// Smoothing parameters
const float alpha = 0.5; // Smoothing factor (0.3-0.6 for faster response)

// Variables
float zeroCurrentVoltage = 0; // Voltage at zero current
float current = 0;            // Current value
float smoothedVoltage = 0;    // Smoothed voltage value

// Function to calibrate the sensor (measures voltage at zero current)
float calibrateSensor() {
  float sum = 0;
  const int samples = 100; // Number of samples for calibration

  for (int i = 0; i < samples; i++) {
    sum += analogRead(ACS712_Pin) * V_Ref / ADC_Resolution * VoltageDividerGain;
    delay(10);
  }

  return sum / samples; // Return average voltage
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi connected!");

  // Init Supabase
  supabase.begin(supabaseUrl, supabaseKey);

  // Calibrate current sensor
  Serial.println("Calibrating sensor...");
  zeroCurrentVoltage = calibrateSensor();
  smoothedVoltage = zeroCurrentVoltage; // Initialize smoothed value
  Serial.println("Calibration completed!");
  Serial.print("Zero current voltage: ");
  Serial.println(zeroCurrentVoltage, 3);

  lastTime = millis();
}

void loop() {

  unsigned long currentTime = millis();
  float timeDiff = (currentTime - lastTime) / 1000.0; // seconds
  lastTime = currentTime;

  // Read voltage from ZMPT101B
  float voltage = analogRead(voltagePin) * (3.3 / 4095.0) *
                  100; // adjust based on ZMPT scaling

  // Calculate Current from ACS712
  // Read raw voltage from the sensor
  float rawVoltage =
      (analogRead(ACS712_Pin) * V_Ref / ADC_Resolution) * VoltageDividerGain;

  // Apply exponential smoothing
  smoothedVoltage = alpha * rawVoltage + (1 - alpha) * smoothedVoltage;

  // Calculate current (voltage relative to zero point)
  current = (smoothedVoltage - zeroCurrentVoltage) / (Sensitivity / 1000.0);

  // Apply calibration factor
  current *= calibrationFactor;
  current = 60 / voltage;

  // // Ignore noise below the threshold
  // if (abs(current) < noiseThreshold) {
  //   current = 0; // Set current to zero
  // }

  // Calculate power and energy
  float power = voltage * current;      // Watts
  float deltaEnergy = power * timeDiff; // Joules
  energy += deltaEnergy;
  totalEnergy += deltaEnergy;

  // Print sensor readings
  Serial.print("V: ");
  Serial.print(voltage, 2);
  Serial.print(" | I: ");
  Serial.print(current, 3);
  Serial.print(" | P: ");
  Serial.print(power, 2);
  Serial.print(" | E: ");
  Serial.print(energy, 2);
  Serial.print(" | Total E: ");
  Serial.println(totalEnergy, 2);

  // Check if energy threshold is reached (360 Joules = 0.1 kWh)
  if (energy >= 360.0) {
    Serial.println("LED blinking - Energy threshold reached!");
    digitalWrite(ledPin, HIGH);

    // change the correct columns names you create in your table
    String jsonData = "{\"total_energy_joules\": " + String(totalEnergy) + " }";

    // sending data to supabase
    int response = supabase.insert(tableName, jsonData, false);
    if (response == 200) {
      Serial.println("Data inserted successfully!");
    } else {
      Serial.println(response);
    }

    delay(500); // LED blink duration
    digitalWrite(ledPin, LOW);
    energy -= 360.0; // Reset energy counter
  }

  delay(100); // Main loop delay
}
