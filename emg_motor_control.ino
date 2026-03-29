#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <uMyo_RF24.h> // We are going back to the RF24 library

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

const int THRESHOLD = 200; //As you do more testing you can change the minimum value
const int MAX_MUSCLE = 400; //Same for here, you can change the maximum value which makes it go the fastest speed
uint32_t last_update_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(1000); 

  // Initialize the Radio (CE=6, CSN=5)
  uMyo.begin(6, 5); 

  if (!AFMS.begin()) {
    Serial.println("Motor Shield not found!");
    while (1);
  }

  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);

  Serial.println("nRF Setup Ready!");
}

void loop() {
  uMyo.run(); 

  if (millis() - last_update_ms > 40) {
    int dev_count = uMyo.getDeviceCount();

    if (dev_count >= 2) {
      int leftHand  = uMyo.getMuscleLevel(0);
      int rightHand = uMyo.getMuscleLevel(1);

      // Left Motor
      if (leftHand > THRESHOLD) {
        int l_speed = map(leftHand, THRESHOLD, MAX_MUSCLE, 0, 255);
        L_MOTOR->setSpeed(constrain(l_speed, 0, 255));
        L_MOTOR->run(FORWARD);
      } else {
        L_MOTOR->run(RELEASE);
      }

      // Right Motor
      if (rightHand > THRESHOLD) {
        int r_speed = map(rightHand, THRESHOLD, MAX_MUSCLE, 0, 255);
        R_MOTOR->setSpeed(constrain(r_speed, 0, 255));
        R_MOTOR->run(FORWARD);
      } else {
        R_MOTOR->run(RELEASE);
      }
    } else {
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);
    }
    last_update_ms = millis();
  }
}