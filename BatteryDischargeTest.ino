// Adding the required libraries
#include <SD.h> 
#include <Wire.h> 

// Using #define because it does not use memory space
#define CS 4
#define ANALOG_IN_PIN A0

File file; 

// Declaring the variables
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0; 
float ref_voltage = 5.0;
int adc_value = 0;
 
void setup() {
 
  Serial.begin(2000000);
  Serial.println("DC Voltage Test");
  pinMode(CS, OUTPUT);
 
  if (!SD.begin(CS)) { 
    Serial.println("Could not initialize SD card.");
  }
 
  if (SD.exists("file.txt")) {
    Serial.println("File exists.");

    if (SD.remove("file.txt") == true) {
      Serial.println("Successfully removed file.");
    } else {
      Serial.println("Could not removed file.");
    }
  }
}

// Only to keep the program running
void loop() {
  readVoltage();
  delay(230);
  writeFile();
}
 
void writeFile() {
  file = SD.open("file.txt", FILE_WRITE);

  if (file) {  
    file.println(in_voltage, 2);
    file.close();
   
  } else {
    Serial.println("Could not open file (writing).");
  }
}
 
void readFile() {
  file = SD.open("file.txt", FILE_READ);

  if (file) {
    Serial.println("--- Reading start ---");

    char character;

    while ((character = file.read()) != -1) {
      Serial.print(character);
    }
    file.close();
    Serial.println("--- Reading end ---");
  } else {
    Serial.println("Could not open file (reading).");
  }
}
 
void readVoltage() {
  float adc_voltage = 0.0, in_voltage = 0.0, R1 = 30000.0, R2 = 7500.0, ref_voltage = 5.0;
  int adc_value = 0;

  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
  in_voltage = adc_voltage / (R2/(R1+R2)); 
   
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);
}

void readCurrent() {
  unsigned int x = 0;
  float AcsValue = 0.0, Samples = 0.0, AcsValueF = 0.0, voltage = 0.0;

  AcsValue = analogRead(A0);
  voltage=AcsValue*(5.0 / 1024.0);         //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts

  Serial.print("Raw Voltage:");
  Serial.print(voltage);

  AcsValueF = (2.5 - voltage)*1000/0.185; //2.5 is offset,,,   0.185v is rise in output voltage when 1A current flows at input
  
  Serial.print("\t");
  Serial.print("Motor Current :");
  Serial.print(AcsValueF);               //Print the read current on Serial monitor
  Serial.println(" mA");
}
