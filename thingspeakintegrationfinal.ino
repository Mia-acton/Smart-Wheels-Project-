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

// WiFi & ThingSpeak initialisation
const char* ssid = "g00428601@atuwifi.ie";      
const char* password = "nEdfx45xn@vc";  
unsigned long channelID = 2786014; 
const char* writeAPIKey = "JQZV5488PE5ZU0GH"; // authenticate and authorise API requests 

WiFiClient client; // client object that handles network communication 

void setup() {
  // Motor Control Setup
  Serial.begin(115200);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  
  // Set up PWM for motor speed control
  ledcAttach(PWM_CHANNEL_E1, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_CHANNEL_E2, PWM_FREQ, PWM_RES);
  ledcAttachChannel(E1, PWM_FREQ, PWM_RES, PWM_CHANNEL_E1);
  ledcAttachChannel(E2, PWM_FREQ, PWM_RES, PWM_CHANNEL_E2);
  
  stopMotors();

  // Sensor Setup
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

  // Alcohol Detection upon initialisation of system 
  bool alcoholDetected = digitalRead(alcoholSensorPin);
  if (alcoholDetected) {
    lcd.print("Alcohol Detected");
    delay(1000);
    lcd.clear();
    lcd.print("System Shutdown.");
    Serial.println("Alcohol Detected! System Shutdown");
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
    delay(1000);
    lcd.clear();
  }

  // WiFi Setup and ThingSpeak initialisation 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  ThingSpeak.begin(client);
}

void loop() {
  // Read the sensors
  int alcoholLevel = analogRead(alcoholSensorPin);  // Reads alcohol sensor
  int flameDetected = digitalRead(flameSensorPin);  // Reads flame sensor
  int gasLevel = analogRead(gasSensorPin);          // Reads gas sensor

  // Update LCD with sensor data
  lcd.setCursor(0, 0);
  lcd.print("Fire: ");
  lcd.print(flameDetected == HIGH ? "STOP" : "Safe"); 
  lcd.setCursor(0, 1);
  lcd.print("Gas: ");
  lcd.print(gasLevel);
  delay(1000); // Update the LCD every second

  // Send data to ThingSpeak
  ThingSpeak.setField(1, alcoholLevel);
  ThingSpeak.setField(2, flameDetected);
  ThingSpeak.setField(3, gasLevel);

  int response = ThingSpeak.writeFields(channelID, writeAPIKey);
  if (response == 200) {
    Serial.println("Data successfully sent to ThingSpeak!");
  } else {
    Serial.println("Error sending data to ThingSpeak: " + String(response));
  }

  // Check for danger (flame or gas level)
  if (isDangerDetected()) {
    stopMotors();
    return;  // exit loop if danger is detected
  }

  // Normal operation: Move forward and backward
  controlMotor1(128, true);  // Move forward at 50% speed
  controlMotor2(128, true);  // Move forward at 50% speed
  delay(5000);  // Move forward for 5 seconds

  if (isDangerDetected()) {
    stopMotors();
    return;  // Stop if danger is detected
  }

  controlMotor1(128, false); // Move backward at 50% speed
  controlMotor2(128, false); // Move backward at 50% speed
  delay(5000);  // Move backward for 5 seconds

  stopMotors();  // Stop after moving forward and backward
  delay(1000);   // Wait before next loop cycle
}

// Function to check for danger (flame or gas threshold)
bool isDangerDetected() {
  int flameDetected = digitalRead(flameSensorPin); // LOW if no flame
  int gasLevel = analogRead(gasSensorPin);

  // Check for flame detection
  if (flameDetected == HIGH) {
    Serial.println("**WARNING** Fire Detected!");
    digitalWrite(alertPin, HIGH);
    delay(500);
    digitalWrite(alertPin, LOW);
    delay(500);
    digitalWrite(alertPin, HIGH);
    delay(500);
    digitalWrite(alertPin, LOW);
    delay(500);
    digitalWrite(alertPin, HIGH);
    return true;  // Danger detected
  } else {
    digitalWrite(alertPin, LOW);
  }

  // Check for gas detection
  if (gasLevel > gasThreshold) {
    Serial.println("**WARNING** Gas Detected!");
    digitalWrite(buzzerPin, HIGH);
    return true;  // Danger detected
  }

  // No danger detected
  digitalWrite(alertPin, LOW);
  digitalWrite(buzzerPin, LOW);
  return false;
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  ledcWrite(PWM_CHANNEL_E1, 0);
  ledcWrite(PWM_CHANNEL_E2, 0);
}

// Function to control Motor 1
void controlMotor1(int speed, bool forward) {
  digitalWrite(M1, forward ? HIGH : LOW);  // Set direction
  ledcWrite(PWM_CHANNEL_E1, speed);       // Set speed
}

// Function to control Motor 2
void controlMotor2(int speed, bool forward) {
  digitalWrite(M2, forward ? HIGH : LOW);  // Set direction
  ledcWrite(PWM_CHANNEL_E2, speed);       // Set speed
}
