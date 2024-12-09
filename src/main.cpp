#include <Arduino.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 26 // CE pin connected to GPIO26
#define CSN_PIN 27 // CSN pin connected to GPIO27
#define TX_LED 2 // LED to indicate transmission (optional)
#define RX_LED 4 // LED to indicate receiving status (blink when no data)

RF24 radio(CE_PIN, CSN_PIN); // Initialize RF24 radio
const byte Address[6] = "00001"; // Address for communication

// L298N Motor driver pins
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26
#define ENA 5
#define ENB 22

unsigned long lastDataReceivedTime = 0; // Track the time when data was last received
const unsigned long noDataTimeout = 1000; // Timeout in ms for no data (1 second)

void setup() {
  Serial.begin(115200);

  // Initialize the LED pins
  pinMode(TX_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);

  // Initialize the motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Set ENA and ENB pins to HIGH to enable motors
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  // Initialize the RF24 radio
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization failed");
    while (1);
  }
  radio.openReadingPipe(1, Address); // Open reading pipe with the specified address
  radio.setPALevel(RF24_PA_HIGH); // Set RF power to high
  radio.startListening(); // Start listening for incoming data

  Serial.println("Receiver is ready...");
}

void moveForward(byte speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving Forward");
  analogWrite(ENA, map(speed, 0, 5, 0, 255)); // Adjust motor speed
  analogWrite(ENB, map(speed, 0, 5, 0, 255));
}

void moveBackward(byte speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Backward");
  analogWrite(ENA, map(speed, 0, 5, 0, 255));
  analogWrite(ENB, map(speed, 0, 5, 0, 255));
}

void moveLeft(byte speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Left");
  analogWrite(ENA, map(speed, 0, 5, 0, 255));
  analogWrite(ENB, map(speed, 0, 5, 0, 255));
}

void moveRight(byte speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning Right");
  analogWrite(ENA, map(speed, 0, 5, 0, 255));
  analogWrite(ENB, map(speed, 0, 5, 0, 255));
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors Stopped");
}

void loop() {
  if (radio.available()) {
    byte receivedData[2];
    radio.read(&receivedData, sizeof(receivedData)); // Read received data

    byte command = receivedData[0]; // Extract command
    byte speedIndex = receivedData[1]; // Extract speed index

    Serial.print("Command: ");
    Serial.print(command);
    Serial.print(" , Speed Index: ");
    Serial.println(speedIndex);

    lastDataReceivedTime = millis(); // Update the last received time

    // Control motors based on received command
    if (command == 1) { // Forward
      moveForward(speedIndex);
    } else if (command == 2) { // Backward
      moveBackward(speedIndex);
    } else if (command == 3) { // Left
      moveLeft(speedIndex);
    } else if (command == 4) { // Right
      moveRight(speedIndex);
    } else { // Stop
      stopMotors();
    }
    digitalWrite(RX_LED, HIGH); // Turn on RX LED while receiving
  } else {
    unsigned long currentTime = millis();
    if (currentTime - lastDataReceivedTime > noDataTimeout) {
      // Blink RX LED if no data received within timeout
      digitalWrite(RX_LED, HIGH);
      delay(100);
      digitalWrite(RX_LED, LOW);
      delay(100);
      Serial.println("No data received");
    }
  }
}
