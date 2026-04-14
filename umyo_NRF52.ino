#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <uMyo_RF24.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

const int THRESHOLD = 200; 
const int MAX_MUSCLE = 400;
uint32_t last_update_ms = 0;

int leftHand_index = -1;
int rightHand_index = -1;
int brake_and_reverse_index = -1;
bool isMoving = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    uMyo.begin(6, 5);
    if(!AFMS.begin()) {
        Serial.println("Motor Shield not found!");
        while (1);
    }
    L_MOTOR->setSpeed(0);
    R_MOTOR->setSpeed(0);
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
    Serial.println("ready");
}

void loop() {
    uMyo.run(); 
    if(millis() - last_update_ms > 40) {
        int dev_count = uMyo.getDeviceCount();

        // for(int i = 0; i < dev_count; i++) {
        //     Serial.print("Pitch: ");
        //     Serial.print(uMyo.getPitch(i));
        //     Serial.print(" ");
        //     Serial.print("Roll: ");
        //     Serial.print(uMyo.getRoll(i));
        //     Serial.println();
        //     delay(1000);
        // }

        // IMU testing with only wrist sensor connected
        if(dev_count == 1) {
            float roll = uMyo.getRoll(0);
            Serial.println(roll);
            // int wrist = uMyo.getMuscleLevel(0);
            // if(wrist > THRESHOLD) {
            if(isMoving) {
                if(roll < 0 && roll > -1.75) {
                    R_MOTOR->setSpeed(255);
                    R_MOTOR->run(FORWARD);
                    L_MOTOR->run(RELEASE);
                }
                else if(roll < -1.75 && roll > -2.75) {
                    int angle = ((roll * -1) - 1.75) * 255;
                    L_MOTOR->setSpeed(constrain(angle, 0, 255));
                    R_MOTOR->setSpeed(255);
                    R_MOTOR->run(FORWARD);
                    L_MOTOR->run(FORWARD);
                    
                }
                //else if ((roll < -2.75 && roll > -3.14) || (roll > 2.75 && roll < 3.14)) {
                //@90
                // else if(roll < -2.75 || roll > 2.75) {
                //     L_MOTOR->setSpeed(255);
                //     L_MOTOR->run(FORWARD);
                //     R_MOTOR->run(FORWARD);
                }
                else if(roll < 2.75 && roll > 1.75) {
                    int angle = (roll - 1.75) * 255;
                    R_MOTOR->setSpeed(constrain(angle, 0, 255));
                    L_MOTOR->setSpeed(255);
                    L_MOTOR->run(FORWARD);
                    R_MOTOR->run(FORWARD);
                }
                // else if(roll < 1.75 && roll > 0) {
                else {
                    L_MOTOR->setSpeed(255);
                    L_MOTOR->run(FORWARD);
                    R_MOTOR->run(RELEASE);
                }
            }
            else {
                              // --- STATIONARY LOGIC (Pivot Turns) ---
              // Tilt right to pivot around the right wheel
              if(roll < 0 && roll > -1.75) { 
                  L_MOTOR->setSpeed(255);
                  L_MOTOR->run(FORWARD);
                  R_MOTOR->run(RELEASE); // Right wheel stays still
              }
              else if(roll < -1.75 && roll > -2.75) {
                    int angle = ((roll * -1) - 1.75) * 255;
                    L_MOTOR->setSpeed(constrain(angle, 0, 255));
                    L_MOTOR->run(FORWARD);
                    R_MOTOR->run(RELEASE); // Right wheel stays still

                    
                }

              else if(roll < 2.75 && roll > 1.75) {
                    int angle = (roll - 1.75) * 255;
                    R_MOTOR->setSpeed(constrain(angle, 0, 255));
                    R_MOTOR->run(FORWARD);
                    L_MOTOR->run(RELEASE); // Left wheel stays still
              }

              // Tilt left to pivot around the left wheel
              else if(roll < 1.75 && roll > 0) {
                  R_MOTOR->setSpeed(255);
                  R_MOTOR->run(FORWARD);
                  L_MOTOR->run(RELEASE); // Left wheel stays still
              }
              // Deadzone: stay completely still
              else {
                  L_MOTOR->run(RELEASE);
                  R_MOTOR->run(RELEASE);
              }
            }
            return;
        }

        if(dev_count >= 3) {
            // Can possibly move this whole sensor assignment section out of the loop if it affects runtime
            // Not needed, but useful if works, can comment out this block to test other things since it isn't finished
            // Make sure sensors are assigned properly if skipping this block
            // Auto identify sensors using each IMU's orientation readings
            if(leftHand_index == -1 || rightHand_index == -1 || brake_and_reverse_index == -1) {
                for(int i = 0; i < 3; i++) {
                    float pitch = uMyo.getPitch(i);
                    float roll = uMyo.getRoll(i);
                    // pitch conditions are place holders, need to use uMyo.getPitch() and uMyo.getRoll and Serial.print them out
                    // to find correct values for the following if statements
                    if(roll > -1 && roll < 1) {
                        brake_and_reverse_index = i; // left arm sensor
                    }
                    else if (roll > 1 || roll < -3) {
                        // can implement pitch val if it needs more to distinguish between right arm sensors
                        leftHand_index = i; // right arm wrist sensor
                    }
                    else { // default assign last sensor
                        rightHand_index = i; // right arm forearm sensor
                    }
                }
                Serial.println("Sensors Assigned!");
                return;
            }

            // Current setup control idea, dual sensor arm controls left and right wheels and speed
            // (with wrist up, wrist down, or tensed arm and palm motions/commands)
            // Single sensor arm will act as a shift-into-reverse when tensed 
            // and will set the dual sensor arm's commands as reverse commands

            int leftHand = uMyo.getMuscleLevel(leftHand_index);
            int rightHand = uMyo.getMuscleLevel(rightHand_index);
            int brake_and_reverse = uMyo.getMuscleLevel(brake_and_reverse_index);

            // Manual "e-brake"
            // Theoretically brakes when readings are crazy high (like physically touching both electrodes on the single sensor arm's sensor)
            if(brake_and_reverse > 500) { // maybe edit this threshold?
                L_MOTOR->run(RELEASE);
                R_MOTOR->run(RELEASE);
            }
            else if(brake_and_reverse > THRESHOLD) { // Reverse
                // Reverses when readings come from both arms
                if(leftHand > THRESHOLD) { // Left Motor
                    int l_speed = map(leftHand, THRESHOLD, MAX_MUSCLE, 0, 255);
                    L_MOTOR->setSpeed(constrain(l_speed, 0, 255));
                    L_MOTOR->run(BACKWARD);
                }
                else {
                    L_MOTOR->run(RELEASE);
                }
                if(rightHand > THRESHOLD) { // Right Motor
                    int r_speed = map(rightHand, THRESHOLD, MAX_MUSCLE, 0, 255);
                    R_MOTOR->setSpeed(constrain(r_speed, 0, 255));
                    R_MOTOR->run(BACKWARD);
                }
                else {
                    R_MOTOR->run(RELEASE);
                }
                
                // Simple reverse if single-sensor arm tenses up
                // int r_speed = map(brake_or_reverse, THRESHOLD, MAX_MUSCLE, 0, 255);
                // int l_speed = map(brake_or_reverse, THRESHOLD, MAX_MUSCLE, 0, 255);
                // R_MOTOR->setSpeed(constrain(r_speed, 0, 255));
                // L_MOTOR->setSpeed(constrain(l_speed, 0, 255));
                // R_MOTOR->run(BACKWARD);
                // L_MOTOR->run(BACKWARD);

                // Simple brake if single-sensor arm tenses up
                // L_MOTOR->run(RELEASE);
                // R_MOTOR->run(RELEASE);
            }
            else { // Forward motions from videos
                if(leftHand > THRESHOLD) { // Left Motor
                    int l_speed = map(leftHand, THRESHOLD, MAX_MUSCLE, 0, 255);
                    L_MOTOR->setSpeed(constrain(l_speed, 0, 255));
                    L_MOTOR->run(FORWARD);
                }
                else {
                    L_MOTOR->run(RELEASE);
                }
                if(rightHand > THRESHOLD) { // Right Motor
                    int r_speed = map(rightHand, THRESHOLD, MAX_MUSCLE, 0, 255);
                    R_MOTOR->setSpeed(constrain(r_speed, 0, 255));
                    R_MOTOR->run(FORWARD);
                }
                else {
                    R_MOTOR->run(RELEASE);
                }
            }
        }
        else { // no input, no movement
            L_MOTOR->run(RELEASE);
            R_MOTOR->run(RELEASE);
        }
        last_update_ms = millis();
    }
}
