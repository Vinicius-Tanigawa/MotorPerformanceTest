#include <Servo.h>

#define THROTTLE_IN_PIN 2 // Assuming the throttle signal is connected to digital pin 2
#define SHUNT_IN_PIN A0
#define VOLTAGE_IN_PIN A2
#define ESC_CONTROL_IN_PIN 9

Servo esc;

// Configuration Constants
const double alpha = 0.5;
const double powerLimitWatt = 220.0; // Adjust as needed
const double batteryCellVoltage = 4.2;
const double batteryMinimumVoltage = 0.88; // For LiPo HV, which is 3.6 V per cell.
const int batteryCellNumber = 6;
const double currentLimitAmps = 75.0;
const double temperatureLimitCelsius = 80.0;

// Filtered Sensor Values
double filteredVoltage = 0.0;
double filteredCurrent = 0.0;

void setup() {
  int throttleValue = 0;

  esc.attach(ESC_CONTROL_IN_PIN);
  initializeESC();

  pinMode(THROTTLE_IN_PIN, INPUT);

  Serial.begin(2000000);
}

void loop() {
  double systemVolt = readVoltage();
  double systemAmpere = readShuntVoltage();

  filteredVoltage = lowPassFilter(systemVolt, filteredVoltage);
  filteredCurrent = lowPassFilter(systemAmpere, filteredCurrent);

  double systemWatt = calculatePower(filteredVoltage, filteredCurrent);

  // Read the throttle value (PWM input)
  int throttleValue = pulseIn(THROTTLE_IN_PIN, HIGH); // Assumes HIGH pulse

  int escValue = calculateESCValue(systemWatt, throttleValue);

  setESCValue(escValue);

  printSensorData(systemVolt, systemAmpere, systemWatt, escValue, filteredVoltage, filteredCurrent);

  delay(20); // 50 Hz
}

// Function to Initialize ESC
void initializeESC() {
  esc.writeMicroseconds(1200); // Set the initial ESC value (adjust as needed for your ESC)
}

// Apply Low-Pass Filter
double lowPassFilter(double value, double filteredValue) {
  return (alpha * filteredValue) + ((1 - alpha) * value);
}

// Calculate Power
double calculatePower(double systemVolt, double systemAmpere) {
  return systemAmpere * systemVolt;
}

// Calculate ESC Value with PID Control
int calculateESCValue(double systemWatt, int throttleValue) {
  // PID constants
  double Kp = 0.9;
  double Ki = 0.01;
  double Kd = 0.001;

  static double previousError = 0;
  static double integral = 0;

  // Calculate error
  double error = powerLimitWatt - systemWatt;
  double proportional = Kp * error;

  integral += Ki * error;
  double derivative = Kd * (error - previousError);

  // Calculate ESC value based on power and throttle input
  int escValue = 1500 + int(proportional + integral + derivative) + throttleValue;
  escValue = constrain(escValue, 1200, 1900); // Adjust the range as needed
  previousError = error;

  return escValue;
}

// Set ESC Value
void setESCValue(int value) {
  esc.writeMicroseconds(value);
}

// Print Sensor Data
void printSensorData(double systemVolt, double systemAmpere, double systemWatt,
                     int escValue, double filteredVolt, double filteredCurrent) {
  Serial.print("\n Current(A): ");
  Serial.println(systemAmpere);

  Serial.print(" Voltage(V): ");
  Serial.println(systemVolt);

  Serial.print(" Power(W): ");
  Serial.println(systemWatt);

  Serial.print(" ESC Value: ");
  Serial.println(escValue);

  Serial.print(" Filtered Voltage(V): ");
  Serial.println(filteredVolt);

  Serial.print(" Filtered Current(A): ");
  Serial.println(filteredCurrent);
}

// Reads voltage from sensor
double readVoltage() {
  double adc_voltage = 0.0, systemVolt = 0.0, R1 = 1000.0, R2 = 4700.0,
         ref_voltage = 5.0;
  int adc_value = 0;

  adc_value = analogRead(VOLTAGE_IN_PIN);
  adc_voltage  = (adc_value * ref_voltage) / 1023.0; 
  systemVolt = adc_voltage / (R2/(R1+R2));

  return systemVolt;
}

// Reads voltage from shunt and transforms to Amperes
double readShuntVoltage() {
  double adc_voltage = 0.0, R1 = 30000.0, R2 = 7500.0, ref_voltage = 5.0,
         shuntVoltage = 0.0, systemAmpere = 0.0;
  int adc_value = 0;

  adc_value = analogRead(SHUNT_IN_PIN);
  adc_voltage  = (adc_value * ref_voltage) / 1023.0; 
  shuntVoltage = adc_voltage / (R2/(R1+R2));
  systemAmpere = shuntVoltage * 2000; // Considering a drop voltage of 0.1 V per 1.0 A

  return systemAmpere;
}