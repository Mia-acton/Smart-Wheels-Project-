// Motor Control Pin Definitions
#define M1  5   // Motor pair 1 control pin (Direction)
#define E1  18  // Motor pair 1 enable pin (PWM for speed)
#define M2  19  // Motor pair 2 control pin (Direction)
#define E2  21  // Motor pair 2 enable pin (PWM for speed)

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
#define PWM_CHANNEL_E1 0  // PWM channel for Motor pair 1 (E1)
#define PWM_CHANNEL_E2 1  // PWM channel for Motor pair 2 (E2)

// LCD Library
#include "rgb_lcd.h"
rgb_lcd lcd;


// Function Declarations
void stopMotors();
void controlMotorPair1(int speed, bool forward);
void controlMotorPair2(int speed, bool forward);

void setup() {
  // Motor Control Setup
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  ledcSetup(PWM_CHANNEL_E1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_E2, PWM_FREQ, PWM_RES);
  ledcAttachPin(E1, PWM_CHANNEL_E1);
  ledcAttachPin(E2, PWM_CHANNEL_E2);
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

  // Alcohol Detection Check
  bool alcoholDetected = digitalRead(alcoholSensorPin);
  if (alcoholDetected) {
    lcd.print("Alcohol Detected");
    delay(1000);
    lcd.clear();
    lcd.print("System Shutdown.");
    Serial.println("Alcohol Detected! System Shutdown.");
    digitalWrite(alertPin, HIGH);
    stopMotors(); // Stop motors if alcohol detected
    while (true) {
      delay(1000);
    }
  } else {
    lcd.print("Safe");
    delay(1000);
    lcd.clear();
    lcd.print("System Active.");
    Serial.println("Safe! System Active.");
    digitalWrite(alertPin, LOW);
    delay(2000);
    lcd.clear();
  }
}

void loop() {
  // Sensor Readings
  int flameDetected = digitalRead(flameSensorPin); // LOW if no flame
  int gasLevel = analogRead(gasSensorPin);

  // Display Sensor Data on LCD
  lcd.setCursor(0, 0); // First row 
  lcd.print("Flame: ");
  lcd.print(flameDetected == LOW ? "Safe" : "STOP"); // if a flame is detected (HIGH), "STOP" will be printed to the serial monitor. If not (LOW), then "Safe" will be printed.

  lcd.setCursor(0, 1);
  lcd.print("Gas: ");
  lcd.print(gasLevel);

  // Flame Detection
  if (flameDetected == HIGH) {
    Serial.println("**WARNING** Fire Detected!");
    digitalWrite(alertPin, HIGH);
    stopMotors(); // Stop motors if flame detected
    delay(1000);
    digitalWrite(alertPin, LOW);
  } else {
    digitalWrite(alertPin, LOW);
  }

  // Gas Detection
  if (gasLevel > gasThreshold) {
    Serial.println("**WARNING** Gas Detected!");
    digitalWrite(buzzerPin, HIGH);
    stopMotors(); // Stop motors if gas detected
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  // Motor Control Logic (if no danger detected)
  if (flameDetected == LOW && gasLevel <= gasThreshold) {
    controlMotorPair1(128, true);  // Move forward at 50% speed
    controlMotorPair2(128, true);
  } else {
    stopMotors(); // Ensure motors are stopped during danger
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
