#include <TMC2209.h>
#include <SoftwareSerial.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/
//
// To make this library work with those boards, refer to this library example:
// examples/UnidirectionalCommunication/SoftwareSerial
// Create a SoftwareSerial instance on two digital pins (choose ones that support change interrupts, e.g., 10 and 11)
// SoftwareSerial tmcSerial(10, 11); // (RX, TX)

const uint8_t STEP_PIN = 2;
const uint8_t DIRECTION_PIN = 4;
const uint32_t STEP_COUNT = 51200*2;
const uint16_t HALF_STEP_DURATION_MICROSECONDS = 100;
const uint16_t STOP_DURATION = 1;
const uint8_t RUN_CURRENT_PERCENT = 100;

// Instantiate TMC2209
TMC2209 stepper_driver;

void setup() {
  // Initialize the driver using SoftwareSerial
  // stepper_driver.setup(tmcSerial);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();
  stepper_driver.enable();

  // Start the SoftwareSerial communication at the desired baud rate
  // tmcSerial.begin(115200);
}

void loop() {
  for (uint32_t i = 0; i < STEP_COUNT * 2; ++i) {
    digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
    delayMicroseconds(HALF_STEP_DURATION_MICROSECONDS);
  }
  digitalWrite(DIRECTION_PIN, !digitalRead(DIRECTION_PIN));
  delay(STOP_DURATION);
}

//