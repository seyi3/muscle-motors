#include <Arduino.h>
#include <arm_math.h>
#include <stream_buffer.h>
#include <queue.h>
#include "uMyo_RF24.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define BLOCK_SIZE 32
#define BUFFER_SIZE BLOCK_SIZE * 2

#define SAMPLES_PER_PACKET 8

#define NUM_UMYO_DEVICES 3

#define IMU_QUEUE_LENGTH 10

#define DEBUG_IMU 0
#define DEBUG_EMG 0

#define SERIAL_PLOTTER_MIN 0
#define SERIAL_PLOTTER_MAX 1

volatile bool thumb = false;
volatile bool mrp = false;

StreamBufferHandle_t xRawDataBuffer;
TaskHandle_t xPollRadioTaskHandle;

uint8_t last_data_id = -1;
bool first_packet = true;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

typedef struct {
  uint8_t channel_id;
  float32_t avg_muscle_level;
  float avg_pitch;
  float avg_roll;
  float avg_yaw;
} ProcessedData_t;

QueueHandle_t xProcessedDataQueue;

typedef struct{
  StreamBufferHandle_t xRawDataBuffer;
  QueueHandle_t xImuDataQueue;
  uint8_t last_data_id; //= -1;
  bool first_packet; //= true;

  float32_t raw_data_block[BLOCK_SIZE];
  float32_t notch_filtered_60Hz_block[BLOCK_SIZE];
  float32_t windowed_block[BLOCK_SIZE];
  float32_t fft_output[BLOCK_SIZE];
  float32_t magnitudes[BLOCK_SIZE / 2];

  arm_biquad_casd_df1_inst_f32 notch_filter_60Hz_instance;
  float32_t notch_filter_60Hz_state[4];

  arm_rfft_fast_instance_f32 fft_instance;

  float32_t avg_muscle_level;
  float avg_pitch;
  float avg_roll;
  float avg_yaw;
} Umyo_Channel_t;

typedef struct {
  float pitch;
  float roll;
  float yaw;
} ImuData_t;

Umyo_Channel_t channels[NUM_UMYO_DEVICES];

float32_t window_array[BLOCK_SIZE];

// Shared Notch Coefficients (60 Hz Notch @ 1000 Hz Fs)
float32_t notch_filter_60Hz_coeffs[5] = {
    0.99479124f, -1.89220538f,  0.99479124f,  
    1.89220538f, -0.98958248f                 
};

uint32_t last_processed_time_ms = 0;

void poll_radio_task(void *pvParameters) {
  int16_t raw_data_int[SAMPLES_PER_PACKET];
  float32_t raw_data_float[SAMPLES_PER_PACKET];
  ImuData_t imu_data;

  while (1) {
    uMyo.run();
    int device_count = uMyo.getDeviceCount();
    
    for (int i = 0; i < device_count && i < NUM_UMYO_DEVICES; i++) {
      uint8_t current_data_id = uMyo.getDataID(i);
      Umyo_Channel_t *channel = &channels[i];

      if (channel->first_packet || current_data_id != channel->last_data_id) {
        channel->last_data_id = current_data_id;
        channel->first_packet = false;

        uMyo.getRawData(i, raw_data_int);

        for (int j = 0; j < SAMPLES_PER_PACKET; j++) {
          raw_data_float[j] = (float32_t)raw_data_int[j];
        }

        xStreamBufferSend(channel->xRawDataBuffer, raw_data_float, sizeof(raw_data_float), 0);

        imu_data.pitch = uMyo.getPitch(i);
        imu_data.roll = uMyo.getRoll(i);
        imu_data.yaw = uMyo.getYaw(i);
        xQueueSend(channel->xImuDataQueue, &imu_data, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void process_raw_data_task(void *pvParameters) {
  int id = (int)pvParameters;
  Umyo_Channel_t *channel = &channels[id];
  ImuData_t imu_data;
  
  while (1) {
    size_t bytes_received = xStreamBufferReceive(channel->xRawDataBuffer, channel->raw_data_block, sizeof(channel->raw_data_block), portMAX_DELAY);
    if (bytes_received == sizeof(channel->raw_data_block)) {
      arm_biquad_cascade_df1_f32(&channel->notch_filter_60Hz_instance, channel->raw_data_block, channel->notch_filtered_60Hz_block, BLOCK_SIZE);
      arm_mult_f32(channel->notch_filtered_60Hz_block, window_array, channel->windowed_block, BLOCK_SIZE);
      arm_rfft_fast_f32(&channel->fft_instance, channel->windowed_block, channel->fft_output, 0);
      arm_cmplx_mag_f32(channel->fft_output, channel->magnitudes, BLOCK_SIZE / 2);

      float32_t sum = 0;
      for (int i = 4; i < 12; i++) {
        sum += channel->magnitudes[i];
      }

      channel->avg_muscle_level *= 0.9;
      channel->avg_muscle_level += 0.1 * (sum);

      float sum_pitch = 0.0f;
      float sum_roll  = 0.0f;
      float sum_yaw   = 0.0f;
      int imu_count   = 0;

      while (xQueueReceive(channel->xImuDataQueue, &imu_data, 0) == pdTRUE) {
        sum_pitch += imu_data.pitch;
        sum_roll  += imu_data.roll;
        sum_yaw   += imu_data.yaw;
        imu_count++;
      }

      if (imu_count > 0) {
        float avg_pitch = sum_pitch / imu_count;
        float avg_roll  = sum_roll / imu_count;
        float avg_yaw   = sum_yaw / imu_count;
        
        channel->avg_pitch *= 0.9;
        channel->avg_pitch += 0.1 * avg_pitch;

        channel->avg_roll *= 0.9;
        channel->avg_roll += 0.1 * avg_roll;

        channel->avg_yaw *= 0.9;
        channel->avg_yaw += 0.1 * avg_yaw;
      }



     
      ProcessedData_t processed_data;
      processed_data.channel_id = id;
      processed_data.avg_muscle_level = channel->avg_muscle_level;
      processed_data.avg_pitch = channel->avg_pitch;
      processed_data.avg_roll = channel->avg_roll;
      processed_data.avg_yaw = channel->avg_yaw;

      //send to processed data queue
      xQueueSend(xProcessedDataQueue, &processed_data, 0);
    }
  }
}

void central_logic_task(void *pvParameters) {
  ProcessedData_t processed_data;
  float32_t v0 = 0.0;
  float32_t v1 = 0.0;

  float32_t cleaned0 = 0.0;
  float32_t cleaned1 = 0.0;

  float32_t a0 = 1.0;
  float32_t b0 = 32;

  float32_t a1 = 1.0;
  float32_t b1 = 32;

  while(1) {
    // Wait for a message from any channel
    if (xQueueReceive(xProcessedDataQueue, &processed_data, portMAX_DELAY) == pdPASS) {
      uint8_t channel_id = processed_data.channel_id;
      float32_t avg_muscle_level = processed_data.avg_muscle_level;
      float avg_pitch = processed_data.avg_pitch;
      float avg_roll = processed_data.avg_roll;
      float avg_yaw = processed_data.avg_yaw;

      if (channel_id == 0) {
        v0 = avg_muscle_level - 1824;
      } else if (channel_id == 1) {
        v1 = avg_muscle_level - 1824;
      }

      if (DEBUG_EMG || DEBUG_IMU) {
        Serial.print("Min:");
        Serial.print(SERIAL_PLOTTER_MIN);
        Serial.print(",Max:");
        Serial.print(SERIAL_PLOTTER_MAX);
        Serial.print(",");
      }

      if (DEBUG_EMG) {
        Serial.print("Channel");
        Serial.print(channel_id);
        Serial.print("Average:");
        Serial.print(avg_muscle_level - 1824);
        Serial.print(",");
      }

      if (DEBUG_IMU) {
        Serial.print("AvgPitch"); 
        Serial.print(channel_id);
        Serial.print(":");
        Serial.print(avg_pitch);
        Serial.print(", AvgRoll"); 
        Serial.print(channel_id);
        Serial.print(":");
        Serial.print(avg_roll);
        Serial.print(", AvgYaw"); 
        Serial.print(channel_id);
        Serial.print(":");
        Serial.print(avg_yaw);
        Serial.print(",");
      }

      cleaned0 =  pow(v0, 2.0) / (a0 * v1 + b0);
      cleaned0 = constrain(cleaned0, 0, 2048);
      thumb = cleaned0 > 256;

      cleaned1 = pow(v1, 2.0) / (a1 * v0 + b1);
      cleaned1 = constrain(cleaned1, 0, 2048);
      mrp = cleaned1 > 256;

      // Serial.print("Thumb:");
      // Serial.print(thumb);
      // Serial.print(",");
      // Serial.print("MRP:");
      // Serial.println(mrp);
    }
  }
}

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10); 
  }

  Serial.println("Initializing channels...");

  for (int i = 0; i < NUM_UMYO_DEVICES; i++) {
    Umyo_Channel_t *channel = &channels[i];

    channel->last_data_id = -1;
    channel->first_packet = true;
    channel->avg_muscle_level = 0;

    channel->xRawDataBuffer = xStreamBufferCreate(BUFFER_SIZE * sizeof(float32_t), BLOCK_SIZE * sizeof(float32_t));
    if (channel->xRawDataBuffer == NULL) {
      Serial.print("Failed to create raw data buffer for channel ");
      Serial.print(i);
      Serial.println("!");
      Serial.println("Halting program...");
      while(1);
    }

    channel->xImuDataQueue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(ImuData_t));
    if (channel->xImuDataQueue == NULL) {
      Serial.print("Failed to create IMU queue for channel ");
      Serial.println(i);
      Serial.println("Halting program...");
      while(1);
    }

    arm_biquad_cascade_df1_init_f32(&channel->notch_filter_60Hz_instance, 1, notch_filter_60Hz_coeffs, channel->notch_filter_60Hz_state);
    arm_rfft_fast_init_f32(&channel->fft_instance, BLOCK_SIZE);

    char task_name[20];
    sprintf(task_name, "ProcessRawDataChannel%d", i);
    xTaskCreate(process_raw_data_task, task_name, 2048, (void*)i, 1, NULL);
  }
  
  for (int i = 0; i < BLOCK_SIZE; i++) {
    window_array[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (BLOCK_SIZE - 1)));
  }

  uMyo.begin();
  AFMS.begin();
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);

  xProcessedDataQueue = xQueueCreate(20, sizeof(ProcessedData_t));
  xTaskCreate(central_logic_task, "LogicTask", 4096, NULL, 2, NULL);

  xTaskCreate(poll_radio_task, "PollRadio", 2048, NULL, 3, &xPollRadioTaskHandle);


  Serial.println("Listening for uMyo packets...");

  // vTaskDelete(NULL); //deletes loop task in underlying 
}

const int THRESHOLD = 200; 
const int MAX_MUSCLE = 400;
uint32_t last_print_ms = 0;

enum CarState { // 3 moving states, not moving, slower speed, fast speed
    STOP,
    SLOW,
    FAST,
    REVERSE
};

CarState currentState = STOP; // initially is in a "stopped" state

bool rising_edge_go = false;
bool rising_edge_stop = false;

void steerCarStopped(float roll, int current_max) {
    if(roll < 0 && roll > -1.75) {
        R_MOTOR->setSpeed(current_max);
        R_MOTOR->run(FORWARD);
        L_MOTOR->run(RELEASE);
    }
    else if(roll < -1.75 && roll > -2.75) {
        int angle = ((roll * -1) - 1.75) * current_max;
        L_MOTOR->setSpeed(constrain(angle, 0, current_max));
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(RELEASE);
        
    }
    else if(roll < -2.75 || roll > 2.75) {
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(RELEASE);
        R_MOTOR->run(RELEASE);
    }
    else if(roll < 2.75 && roll > 1.75) {
        int angle = (roll - 1.75) * current_max;
        R_MOTOR->setSpeed(constrain(angle, 0, current_max));
        R_MOTOR->run(FORWARD);
        L_MOTOR->run(RELEASE);
    }
    // else if(roll < 1.75 && roll > 0) {
    else {
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(RELEASE);
    }
}

void steerCarForward(float roll, int current_max) {
    if(roll < 0 && roll > -1.75) {
        R_MOTOR->setSpeed(current_max);
        R_MOTOR->run(FORWARD);
        L_MOTOR->run(RELEASE);
    }
    else if(roll < -1.75 && roll > -2.75) {
        int angle = ((roll * -1) - 1.75) * current_max;
        L_MOTOR->setSpeed(constrain(angle, 0, current_max));
        R_MOTOR->setSpeed(current_max);
        R_MOTOR->run(FORWARD);
        L_MOTOR->run(FORWARD);
        
    }
    else if(roll < -2.75 || roll > 2.75) {
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
    }
    else if(roll < 2.75 && roll > 1.75) {
        int angle = (roll - 1.75) * current_max;
        R_MOTOR->setSpeed(constrain(angle, 0, current_max));
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
    }
    // else if(roll < 1.75 && roll > 0) {
    else {
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(RELEASE);
    }
}

void steerCarBackward(float roll, int current_max) {
    if(roll < 0 && roll > -1.75) {
        R_MOTOR->setSpeed(current_max);
        R_MOTOR->run(BACKWARD);
        L_MOTOR->run(RELEASE);
    }
    else if(roll < -1.75 && roll > -2.75) {
        int angle = ((roll * -1) - 1.75) * current_max;
        L_MOTOR->setSpeed(constrain(angle, 0, current_max));
        R_MOTOR->setSpeed(current_max);
        R_MOTOR->run(BACKWARD);
        L_MOTOR->run(BACKWARD);
    }
    else if(roll < -2.75 || roll > 2.75) {
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);
    }
    else if(roll < 2.75 && roll > 1.75) {
        int angle = (roll - 1.75) * current_max;
        R_MOTOR->setSpeed(constrain(angle, 0, current_max));
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);
    }
    // else if(roll < 1.75 && roll > 0) {
    else {
        L_MOTOR->setSpeed(current_max);
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(RELEASE);
    }
}

void loop() {
    if(millis() - last_processed_time_ms > 40) {
        int dev_count = uMyo.getDeviceCount();

        float wrist_direction = uMyo.getRoll(2); // wrist sensor for arm rotation angle
        float height_activated = uMyo.getPitch(2); // wrist sensor for arm level/height

        // live roll updates in the serial monitor for directional control help/feedback
        // Serial.println(wrist_direction);

        if(dev_count == 3) { // check to make sure all devices are connected
            if(height_activated > 1) { // checks to make sure arm is in the "controlling" position, manual ebrake
                if(thumb && !rising_edge_go) { // if thumb is received this loop but wasn't in the last loop
                    if(currentState == STOP || currentState == REVERSE) { // up the speed from stopped to slow
                        currentState = SLOW;
                    } 
                    else if(currentState == SLOW) { // up the speed from slow to fast
                        currentState = FAST;
                    } // otherwise speed is at "fast" already and stays there
                }
                else if(mrp && !rising_edge_stop) { // if the mrp signal is received, stop the forward/backward motion
                    if(currentState == STOP) {
                        currentState = REVERSE;
                    }
                    else {
                        currentState = STOP;
                    }
                }

                // switch statement for stop, slow, and forward states
                switch(currentState) {
                    case STOP:
                        L_MOTOR->run(RELEASE);
                        R_MOTOR->run(RELEASE);
                        steerCarStopped(wrist_direction, 128);
                        break;
                    case SLOW:
                        L_MOTOR->setSpeed(128);
                        R_MOTOR->setSpeed(128);
                        L_MOTOR->run(FORWARD);
                        R_MOTOR->run(FORWARD);
                        steerCarForward(wrist_direction, 128);
                        break;
                    case FAST:
                        L_MOTOR->setSpeed(255);
                        R_MOTOR->setSpeed(255);
                        L_MOTOR->run(FORWARD);
                        R_MOTOR->run(FORWARD);
                        steerCarForward(wrist_direction, 255);
                        break;
                    case REVERSE:
                        L_MOTOR->setSpeed(128);
                        R_MOTOR->setSpeed(128);
                        L_MOTOR->run(BACKWARD);
                        R_MOTOR->run(BACKWARD);
                        steerCarBackward(wrist_direction, 128);
                        break;
                }

                // stores if the go signal was received
                if(thumb) {
                    rising_edge_go = true;
                }
                else {
                    rising_edge_go = false;
                }

                if(mrp) {
                    rising_edge_stop = true;
                }
                else {
                    rising_edge_stop = false;
                }
            }
            else { // if arm is by your side/in the "off" position
                L_MOTOR->run(RELEASE);
                R_MOTOR->run(RELEASE);
                currentState = STOP;
                rising_edge_go = false;
            }
        }
        else { // no input, no movement
            // Serial.println("Less than 3 sensors connected, searching for sensors...");
            L_MOTOR->run(RELEASE);
            R_MOTOR->run(RELEASE);
        }

        last_processed_time_ms = millis();
    }
}
