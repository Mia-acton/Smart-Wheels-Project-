// Speed Sesnor Test file 
#include <WiFi.h>
#include <ThingSpeak.h>
#include "rgb_lcd.h"

// Define Motor Control Pins
#define M1  5   // Motor 1 control pin (Direction)
#define E1  18  // Motor 1 enable pin (PWM for speed)
#define M2  19  // Motor 2 control pin (Direction)
#define E2  27  // Motor 2 enable pin (PWM for speed)

// Sensor Pin Definitions
const int flameSensorPin = 34;    // Pin connected to flame sensor OUT
const int gasSensorPin = 35;      // Analog pin connected to MQ2 sensor (AO pin)
const int alertPin = 25;          // Pin connected to LED for flame and alcohol detection
const int buzzerPin = 26;         // Pin connected to buzzer for gas
const int alcoholSensorPin = 2;   // Pin connected to MQ3 alcohol sensor

// Threshold value for gas detection
const int gasThreshold = 400;     // Adjust based on environment and gas level

// PWM parameters
#define PWM_FREQ     5000 // Frequency for PWM
#define PWM_RES      8    // 8-bit resolution (0-255)
#define PWM_CHANNEL_E1 0  // PWM channel for Motor 1 (E1)
#define PWM_CHANNEL_E2 1  // PWM channel for Motor 2 (E2)

// LCD Library
rgb_lcd lcd;
// Pin for the IR speed sensor
const int speedSensorPin = 32; 

// Variable to count pulses from the IR sensor
volatile int pulseCount = 0;

// Variable to store speed (in RPM)
int speedRPM = 0;

// Time-related variables
unsigned long lastMillis = 0;
const unsigned long updateInterval = 1000;  // Update speed every 1 second (1000 ms)

// Interrupt Service Routine (ISR) for counting pulses
void IRAM_ATTR pulseISR() {
  pulseCount++;  // Increment pulse count on every pulse detected by the IR sensor
}

void setup() {
  // Motor Control Setup 
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  ledcAttach(PWM_CHANNEL_E1, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_CHANNEL_E2, PWM_FREQ, PWM_RES);
  ledcAttachChannel(E1, PWM_FREQ, PWM_RES, PWM_CHANNEL_E1);
  ledcAttachChannel(E2, PWM_FREQ, PWM_RES, PWM_CHANNEL_E2);
  stopMotors();

  // Sensor Setup 
  Serial.begin(115200);
  pinMode(flameSensorPin, INPUT);
  pinMode(gasSensorPin, INPUT);
  pinMode(alcoholSensorPin, INPUT);
  pinMode(alertPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(alertPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // LCD Setup 
  lcd.begin(16, 2);
  lcd.print("Initialising...");
  delay(2000);
  lcd.clear();

  // Set up the speed sensor pin and interrupt
  pinMode(speedSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(speedSensorPin), pulseISR, RISING); // Interrupt on rising edge

  // Initial speed check (optional)
  lcd.print("Speed: 0 RPM");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Update the speed every 1 second
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= updateInterval) {
    // Calculate RPM (pulses per second * 60 for RPM)
    speedRPM = (pulseCount * 60) / 1;  // Assumes 1 second interval

    // Display speed on LCD
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");
    lcd.print(speedRPM);  // Print the speed (RPM)
    lcd.print(" RPM");

    // Reset pulse count for the next interval
    pulseCount = 0;
    lastMillis = currentMillis;
  }

  int flameDetected = digitalRead(flameSensorPin);
  int gasLevel = analogRead(gasSensorPin);
  lcd.setCursor(0, 1);
  lcd.print("Gas: ");
  lcd.print(gasLevel);

  // Flame Detection 
  if (flameDetected == HIGH) {
    Serial.println("**WARNING** Fire Detected!");
    digitalWrite(alertPin, HIGH);
    stopMotors();
    delay(1000);
    digitalWrite(alertPin, LOW);
  }

  // Gas Detection 
  if (gasLevel > gasThreshold) {
    Serial.println("**WARNING** Gas Detected!");
    digitalWrite(buzzerPin, HIGH);
    stopMotors();
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  // Motor Control Logic 
  if (flameDetected == LOW && gasLevel <= gasThreshold) {
    controlMotorPair1(128, true);  
    controlMotorPair2(128, true);
  } else {
    stopMotors();
  }

  delay(500);
}

// Function to stop both motor pairs 
void stopMotors() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  ledcWrite(PWM_CHANNEL_E1, 0);
  ledcWrite(PWM_CHANNEL_E2, 0);
}

// Function to control Motor pair 1 (Motors 1 and 3) 
void controlMotorPair1(int speed, bool forward) {
  digitalWrite(M1, forward ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL_E1, speed);
}

// Function to control Motor pair 2 (Motors 2 and 4) 
void controlMotorPair2(int speed, bool forward) {
  digitalWrite(M2, forward ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL_E2, speed);
}
