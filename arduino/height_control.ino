#include <TMC2209.h>
// #include <SoftwareSerial.h>  // Uncomment if you later need UART configuration

// Define pins and constants
const uint8_t STEP_PIN = 2;
const uint8_t DIRECTION_PIN = 4;
const uint32_t STEPS_PER_COMMAND = 51200;  // Number of full steps per command
const uint16_t STEP_INTERVAL_MICROS = 100; // Delay between each step toggle (microseconds)
const uint8_t RUN_CURRENT_PERCENT = 100;

// Instantiate TMC2209 driver
TMC2209 stepper_driver;

// Global variables for non-blocking stepping
volatile long stepsRemaining = 0;    // Total toggles remaining (each full step requires 2 toggles)
unsigned long lastStepTime = 0;
bool paused = false;                 // Pause state flag

void setup() {
  Serial.begin(115200);
  Serial.println("Press 'u' to move up, 'd' to move down, 'p' to toggle pause.");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  
  // Set a default direction
  digitalWrite(DIRECTION_PIN, LOW);

  // Configure the stepper driver
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();
  stepper_driver.enable();
}

void loop() {
  // Process serial input to change motion commands or pause
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'u') {
      Serial.println("Key 1 pressed: Moving Up.");
      digitalWrite(DIRECTION_PIN, HIGH);  // Set direction for "up" (adjust if needed)
      stepsRemaining = STEPS_PER_COMMAND * 2; // Each full step requires 2 toggles
    } 
    else if (input == 'd') {
      Serial.println("Key 2 pressed: Moving Down.");
      digitalWrite(DIRECTION_PIN, LOW);   // Set direction for "down"
      stepsRemaining = STEPS_PER_COMMAND * 2;
    } 
    else if (input == 'p') {
      paused = !paused;
      if (paused) {
        Serial.println("Paused.");
      } else {
        Serial.println("Resumed.");
      }
    } 
    else {
      Serial.println("Invalid key. Please press 'u', 'd', or 'p'.");
    }
  }
  
  // Non-blocking stepping: only execute steps if not paused
  if (!paused && stepsRemaining > 0) {
    unsigned long currentTime = micros();
    if (currentTime - lastStepTime >= STEP_INTERVAL_MICROS) {
      // Toggle the STEP_PIN to generate a step pulse
      digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
      lastStepTime = currentTime;
      stepsRemaining--;  // Count one toggle
    }
  }
}
