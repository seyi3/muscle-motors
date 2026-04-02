#include <Arduino.h>
#include <arm_math.h>
#include <stream_buffer.h>
#include "uMyo_RF24.h"

#define BLOCK_SIZE 32 //samples per block for FFT
#define UMYO_DEVICE_INDEX 0

StreamBufferHandle_t xRawDataBuffer;
TaskHandle_t xProcessRawDataTaskHandle;
TaskHandle_t xPollRadioTaskHandle;

uint8_t last_data_id = -1;
bool first_packet = true;

arm_biquad_casd_df1_inst_f32 notch_filter_60Hz_instance;
float32_t notch_filter_60Hz_state[4];

arm_rfft_fast_instance_f32 fft_instance;

float32_t raw_data_block[BLOCK_SIZE];
float32_t notch_filtered_60Hz_block[BLOCK_SIZE];
float32_t windowed_block[BLOCK_SIZE];
float32_t fft_output[BLOCK_SIZE];
float32_t magnitudes[BLOCK_SIZE / 2];

float32_t avg_muscle_level = 0;

float32_t window_array[BLOCK_SIZE];

// Shared Notch Coefficients (50 Hz Notch @ 1000 Hz Fs)
float32_t notch_filter_60Hz_coeffs[5] = {
    0.99479124f, -1.89220538f,  0.99479124f,  
    1.89220538f, -0.98958248f                 
};

uint32_t last_processed_time_ms = 0;

void poll_radio_task(void *pvParameters) {
  int16_t raw_int_buffer[8];
  float32_t raw_float_buffer[8];

  while (1) {
    uMyo.run();
    if (uMyo.getDeviceCount() > 0) {
      uint8_t current_data_id = uMyo.getDataID(UMYO_DEVICE_INDEX);
      if (first_packet || current_data_id != last_data_id) {
        last_data_id = current_data_id;
        first_packet = false;

        uMyo.getRawData(UMYO_DEVICE_INDEX, raw_int_buffer);

        for (int i = 0; i < 8; i++) {
          raw_float_buffer[i] = (float32_t)raw_int_buffer[i];
        }
        xStreamBufferSend(xRawDataBuffer, raw_float_buffer, sizeof(raw_float_buffer), 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void process_raw_data_task(void *pvParameters) {
  while (1) {
    size_t bytes_received = xStreamBufferReceive(xRawDataBuffer, raw_data_block, sizeof(raw_data_block), portMAX_DELAY);
    
    if (bytes_received == sizeof(raw_data_block)) {
      arm_biquad_cascade_df1_f32(&notch_filter_60Hz_instance, raw_data_block, notch_filtered_60Hz_block, BLOCK_SIZE);
      arm_mult_f32(notch_filtered_60Hz_block, window_array, windowed_block, BLOCK_SIZE);
      arm_rfft_fast_f32(&fft_instance, windowed_block, fft_output, 0);
      arm_cmplx_mag_f32(fft_output, magnitudes, BLOCK_SIZE / 2);

      float32_t sum = 0;
      for (int i = 4; i < 12; i++) {
        sum += magnitudes[i];
      }

      avg_muscle_level *= 0.9;
      avg_muscle_level += 0.1 * (sum);
      uint32_t current_ms = millis();
      Serial.print("Period:");
      Serial.print(current_ms - last_processed_time_ms);
      last_processed_time_ms = current_ms;

      Serial.print(",Min:0,Max:4096,");
      Serial.print("Average:");
      Serial.println(avg_muscle_level);
    }
  }
}

void setup() {
  Serial.begin(115200);

  Serial.println("Initializing...");

  const size_t raw_data_buffer_size_bytes = sizeof(float32_t) * BLOCK_SIZE * 2;
  xRawDataBuffer = xStreamBufferCreate(raw_data_buffer_size_bytes, sizeof(raw_data_block));
  if (xRawDataBuffer == NULL) {
    Serial.println("Failed to create Stream Buffer! Halting.");
    while(1);
  }

  arm_biquad_cascade_df1_init_f32(&notch_filter_60Hz_instance, 1, notch_filter_60Hz_coeffs, notch_filter_60Hz_state);
  arm_rfft_fast_init_f32(&fft_instance, BLOCK_SIZE);
  for (int i = 0; i < BLOCK_SIZE; i++) {
    window_array[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (BLOCK_SIZE - 1)));
  }

  uMyo.begin();

  xTaskCreate(poll_radio_task, "PollRadio", 2048, NULL, 2, &xPollRadioTaskHandle);
  xTaskCreate(process_raw_data_task, "ProcessRawData", 2048, NULL, 1, &xProcessRawDataTaskHandle);

  Serial.println("Listening for uMyo packets...");

  vTaskDelete(NULL); //deletes loop task in underlying 
}

uint32_t last_print_ms = 0;

void loop() 
{
  //freertos task for loop deleted in setup
}
