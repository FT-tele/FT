

//esp32 s3

#define AUDIO_SPEAKER_BCLK 17  // Speaker BCLK pin
#define AUDIO_SPEAKER_LRC 16   // Speaker LRC (WS) pin
#define AUDIO_SPEAKER_DIN 18   // Speaker DIN pin
#define AUDIO_MIC_SCK 7        // Microphone SCK pin
#define AUDIO_MIC_WS 6         // Microphone WS pin
#define AUDIO_MIC_SD 4         // Microphone SD pin


#define AUDIO_SAMPLE_RATE 48000  //48000  // 48kHz sample rate
#define AUDIO_OPUS_BITRATE 8000
#define AUDIO_OPUS_COMPLEXITY 0

#define FRAME_MS 60
#define CHANNELS 1

// Opus and ring buffer configuration
#define MAX_OPUS_BYTES 128  // Must be power of 2
#define RING_SIZE 31
#define FRAME_SIZE 2880  // 2880 samples per frame
#define OPUS_SIZE 1


bool initI2S();

void phoneTask(void *pvParameters);

void listenTask(void *pvParameters);