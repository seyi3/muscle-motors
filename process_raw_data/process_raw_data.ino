#include <Arduino.h>
#include <arm_math.h>
#include <stream_buffer.h>
#include "uMyo_RF24.h"

#define BLOCK_SIZE 32
#define BUFFER_SIZE BLOCK_SIZE * 2

#define SAMPLES_PER_PACKET 8

#define NUM_UMYO_DEVICES 3

StreamBufferHandle_t xRawDataBuffer;
TaskHandle_t xPollRadioTaskHandle;

uint8_t last_data_id = -1;
bool first_packet = true;

typedef struct{
  StreamBufferHandle_t xRawDataBuffer;
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
} Umyo_Channel_t;

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
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void process_raw_data_task(void *pvParameters) {
  int id = (int)pvParameters;
  Umyo_Channel_t *channel = &channels[id];
  
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

      channel->avg_muscle_level *= 0.7;
      channel->avg_muscle_level += 0.3 * (sum);
      
      Serial.print(",Min:-64,Max:64,");
      Serial.print("Channel");
      Serial.print(id);
      Serial.print("Average:");
      Serial.println(channel->avg_muscle_level - 1832);
    }
  }
}

void setup() {
  Serial.begin(115200);

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

  xTaskCreate(poll_radio_task, "PollRadio", 2048, NULL, 2, &xPollRadioTaskHandle);


  Serial.println("Listening for uMyo packets...");

  vTaskDelete(NULL); //deletes loop task in underlying 
}

uint32_t last_print_ms = 0;

void loop() 
{
  //freertos task for loop deleted in setup
}
