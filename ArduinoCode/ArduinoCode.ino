#include <SoftwareSerial.h>

const int trig = 12;
const int echo = 13;
const int redLED = 9;
const int greenLED = 8;
const int relayPin = 7;
const int buzzerPin = 6;

// Define software serial pins for ESP32-CAM communication
#define ARD_RX_PIN 2 
#define ARD_TX_PIN 3 
SoftwareSerial ESPSerial(ARD_RX_PIN, ARD_TX_PIN);

// Define software serial pins for SIM900 GSM module
#define SIM_RX 10
#define SIM_TX 11
SoftwareSerial SIM900(SIM_RX, SIM_TX);

int duration = 0;
int distance = 0;
unsigned long lastHeartbeat = 0;
bool cameraConnected = false;
bool isAuthorized = false;

// Phone number to send SMS alerts
String phoneNumber = "+94765226321";

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  // Default state - locked
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(relayPin, HIGH);
  digitalWrite(buzzerPin, LOW);
  
  Serial.begin(9600); 
  ESPSerial.begin(9600);
  SIM900.begin(9600);

  Serial.println("Arduino initialized. Waiting for signals from ESP32-CAM...");
  
  // Blink LEDs to indicate system startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    delay(200);
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
    delay(200);
  }
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  
  Serial.println("GSM Module Initializing...");
  SIM900.println("AT");
  delay(1000);
}

void loop() {
  // Check for signal from ESP32-CAM
  static String message = "";
  while (ESPSerial.available() > 0) {
    char c = ESPSerial.read();
    if (c == '\n') {
      processMessage(message);
      message = "";
    } else {
      message += c;
    }
  }

  // Ultrasonic Sensor logic
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH, 30000);

  if (duration == 0) {
    Serial.println("No Echo Received ❌");
  } else {
    distance = (duration / 2) / 29.1;
    Serial.print("Distance: ");
    Serial.println(distance);

    //Door logic
    if (distance < 7) {
      if (isAuthorized) {
        
        unlockDoor();
        Serial.println("Authorized person is near the door - Door Unlocked");
        sendSMS("Authorized person unlocked the door.");
      } else {
        
        Serial.println("Unauthorized access detected!");
        unauthorizedAccess();
      }
    } else {
      
      lockDoor();
      Serial.println("No one is near the door - Door Locked");
    }
  }
  
  delay(200);
}

void processMessage(String message) {
  Serial.print("Received message: ");
  Serial.println(message);

  // Ignore corrupted messages
  if (message.length() != 1) {
    Serial.println("Ignoring corrupted or debug message");
    return;
  }

  if (message == "U") {
    isAuthorized = true;
    Serial.println("Person recognized by ESP32-CAM");
  } else if (message == "L") {
    isAuthorized = false;
    Serial.println("Person not recognized by ESP32-CAM");
  } else if (message == "T") {
    Serial.println("Test signal received - ESP32-CAM is starting up");
    cameraConnected = true;
    blinkLEDs(2);
  } else if (message == "H") {
    Serial.println("Heartbeat received from ESP32-CAM");
    lastHeartbeat = millis();
    cameraConnected = true;
  } else {
    Serial.print("Unknown message received: ");
    Serial.println(message);
  }
}

// Function to unlock door
void unlockDoor() {
  digitalWrite(relayPin, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, LOW);
  digitalWrite(buzzerPin, LOW);
}

// Function to lock door
void lockDoor() {
  digitalWrite(relayPin, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, HIGH);
  digitalWrite(buzzerPin, LOW);
}

// Function for unauthorized access
void unauthorizedAccess() {
  digitalWrite(relayPin, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, HIGH);
  digitalWrite(buzzerPin, HIGH);
  delay(1000);
  digitalWrite(buzzerPin, LOW);
  sendSMS("Unauthorized person tried to unlock the door!");
}

// Function to send SMS
void sendSMS(String message) {
  Serial.print("Sending SMS: ");
  Serial.println(message);

  SIM900.println("AT+CMGF=1");
  delay(1000);
  SIM900.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(1000);
  SIM900.print(message);
  delay(1000);
  SIM900.write(26);
  delay(3000);
  Serial.println("SMS Sent!");
}

// Function to blink both LEDs
void blinkLEDs(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
    delay(100);
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    delay(100);
  }
  
  // Return to default state
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
}
