#include <Arduino.h>

#include <driver/i2s.h>
#include <opus.h>
#include "Phone.h"

#include "AES.h"

#include "VariableDefine.h"



uint8_t cipherAudio = 0;
 

int16_t audio_buffer[FRAME_SIZE];
int16_t opus_pcm[FRAME_SIZE];

// Speaker I2S configuration
i2s_config_t i2s_speaker_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = AUDIO_SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  .intr_alloc_flags = 0,
  .dma_buf_count = 8,
  .dma_buf_len = 512,
  .use_apll = false,
  .tx_desc_auto_clear = true,
  .fixed_mclk = 0
};

// Speaker I2S pin configuration
i2s_pin_config_t i2s_speaker_pin_config = {
  .mck_io_num = -1,
  .bck_io_num = AUDIO_SPEAKER_BCLK,
  .ws_io_num = AUDIO_SPEAKER_LRC,
  .data_out_num = AUDIO_SPEAKER_DIN,
  .data_in_num = I2S_PIN_NO_CHANGE
};

// Microphone I2S configuration
i2s_config_t i2s_mic_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = AUDIO_SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags = 0,
  .dma_buf_count = 8,
  .dma_buf_len = 512
};

// Microphone I2S pin configuration
i2s_pin_config_t i2s_mic_pin_config = {
  .mck_io_num = -1,
  .bck_io_num = AUDIO_MIC_SCK,
  .ws_io_num = AUDIO_MIC_WS,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = AUDIO_MIC_SD
};



//-------------------------------------------------------api


bool initI2S() {

  // Initialize I2S driver for microphone
  if (i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL) != ESP_OK) {
    //Serial.println("Failed to install I2S mic driver");
    return false;
  }
  if (i2s_set_pin(I2S_NUM_0, &i2s_mic_pin_config) != ESP_OK) {
    //Serial.println("Failed to set I2S mic pins");
    return false;
  }
  // Initialize I2S driver for speaker
  if (i2s_driver_install(I2S_NUM_1, &i2s_speaker_config, 0, NULL) != ESP_OK) {
    //Serial.println("Failed to install I2S speaker driver");
    return false;
  }
  if (i2s_set_pin(I2S_NUM_1, &i2s_speaker_pin_config) != ESP_OK) {
    //Serial.println("Failed to set I2S speaker pins");
    return false;
  }
  //Serial.println("  I2S   init success \n");
  return true;
}


//-------------------------------------------------------task
// Audio processing task
void phoneTask(void* pvParameters) {



  size_t written;



  volatile uint8_t speak_index = 0;

  uint8_t frameOffset = 0;
  uint8_t sndIndex = 0;  //wrap into sndPkt.payload
  volatile uint8_t speak_offset = OPUS_SIZE + 1;

  OpusEncoder* encoder;


  // Initialize Opus encoder
  int error;
  encoder = opus_encoder_create(AUDIO_SAMPLE_RATE, CHANNELS, OPUS_APPLICATION_VOIP, &error);
  if (error != OPUS_OK) {
    //Serial.printf("Opus encoder creation failed: %s\n", opus_strerror(error));
  }

  error = opus_encoder_init(encoder, AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP);
  if (error != OPUS_OK) {
    //Serial.printf("Opus encoder  init failed: %s\n", opus_strerror(error));
  }
  opus_encoder_ctl(encoder, OPUS_SET_BITRATE(AUDIO_OPUS_BITRATE));
  opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(AUDIO_OPUS_COMPLEXITY));
  opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

  size_t samples_read = 0;
  size_t bytes_read = 0;
  int encoded_bytes = 0;
  esp_err_t read_result;

  uint8_t encoded_frame[DTM];
  uint8_t framePkt[PKT];

  uint8_t cipher_len = 0;




  uint8_t speakKey[KEY];
  uint8_t speakIV[HKEY];
  uint8_t tmpIV[HKEY];


 



  Serial.println("phoneTask initialized");



  while (1) {


    // Read exactly one frame from microphone
    if (ButtonIR) {


      read_result = i2s_read(I2S_NUM_0, audio_buffer, FRAME_SIZE * sizeof(int16_t), &bytes_read, portMAX_DELAY);
      if (read_result != ESP_OK) {
        //Serial.println("Error reading from I2S microphone");
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      samples_read = bytes_read / sizeof(int16_t);
      if (samples_read < FRAME_SIZE) {
        //Serial.println("Incomplete frame read");
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }





      // Encode the frame
      encoded_bytes = opus_encode(encoder, audio_buffer, FRAME_SIZE, encoded_frame, DTM);
      if (encoded_bytes < 0) {
        //Serial.printf("Opus encoding failed: %s\n", opus_strerror(encoded_bytes));
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }



      if (encoded_bytes > 40) {

        if (speak_index == 0) {

          framePkt[0] = encoded_bytes;
          memcpy(&framePkt[2], encoded_frame, encoded_bytes);
          frameOffset = encoded_bytes + 2;
        } else {
          framePkt[1] = encoded_bytes;
          memcpy(&framePkt[frameOffset], encoded_frame, encoded_bytes);
          frameOffset = frameOffset + encoded_bytes;
          SndPkt[sndIndex].payloadData[0] = 1;


          if (cipherAudio == 1) {
            memcpy(tmpIV, speakIV, HKEY);
            aes_cipher(framePkt, frameOffset, &SndPkt[sndIndex].payloadData[1], speakKey, tmpIV, &cipher_len);
            SndPkt[sndIndex].pktLen = cipher_len + 1;
          } else {
            memcpy(&SndPkt[sndIndex].payloadData[1], framePkt, frameOffset);
            SndPkt[sndIndex].pktLen = frameOffset + 1;
          }

          xQueueSend(loraQueue, &sndIndex, portMAX_DELAY);

          sndIndex = (sndIndex + 1) & RING_SIZE;
        }

        speak_index = (speak_index + 1) & OPUS_SIZE;
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Audio processing task
void listenTask(void* pvParameters) {
  // Buffers

  uint8_t listenIdx;

  OpusDecoder* decoder;


  //int len = 0;
  int samples = 0;
  int err;
  decoder = opus_decoder_create(AUDIO_SAMPLE_RATE, CHANNELS, &err);
  if (err != OPUS_OK) {
    //Serial.printf("Opus decoder creation failed: %s\n", opus_strerror(err));
  }



  esp_err_t write_result;
  size_t bytes_written = 0;

  Serial.println(" listen task initialized");
  //vTaskSuspend(NULL);
  while (1) {

    if (xQueueReceive(listenQueue, &listenIdx, portMAX_DELAY)) {
      Serial.printf("\n listenIdx :%d ", listenIdx);

      samples = opus_decode(decoder, frame_buffer[listenIdx], frame_len[listenIdx], opus_pcm, FRAME_SIZE, 0);
      if (samples < 0) {
        Serial.printf("Opus decoding failed: %s\n", opus_strerror(samples));
      } else {



        write_result = i2s_write(I2S_NUM_1, opus_pcm, samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        if (write_result != ESP_OK) {
          //Serial.println("Error writing to I2S speaker");
        }
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
