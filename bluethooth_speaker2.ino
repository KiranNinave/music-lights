#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorAAC.h"
#include "AudioOutputI2S.h"
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <arduinoFFT.h>

#define DEVICE_NAME "SurajKiKirne"

static const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = 44100, // corrected by info from bluetooth
        .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
};

// FFT Settings
arduinoFFT FFT = arduinoFFT();
#define NUM_BANDS 8
#define SAMPLES 512
#define SAMPLING_FREQUENCY 44100
int32_t peak[] = {0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];
float amplitude = 200.0;

// LED Settings
#define LED_COUNT   64 // should be in multiple of 8
#define DATA_PIN 15
int BRIGHTNESS = 100;
int PATTERN = 0;
int TOTAL_PATTERN = 2;
CRGB leds[LED_COUNT];

// push button settings
#define BRIGHTNESS_BUTTON 34 
int brightnessButtonState = LOW;
int brightnessButtonCurrentState;
#define PATTERN_BUTTON 35
int patternButtonState = LOW;
int patternButtonCurrentState;

BluetoothA2DPSink a2dp_sink;

QueueHandle_t queue;

int16_t sample_l_int;
int16_t sample_r_int;

void audio_data_callback(const uint8_t *data, uint32_t len) {
  int item = 0;
  // Only prepare new samples if the queue is empty
  if (uxQueueMessagesWaiting(queue) == 0) {
    int byteOffset = 0;
    for (int i = 0; i < SAMPLES; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }
    // Tell the task in core 1 that the processing can start
    xQueueSend(queue, &item, portMAX_DELAY);
  }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *) {
  Serial.println(state);
}

void createBands(int i, int dsize) {
  uint8_t band = 0;
  if (i <= 2) {
    band = 0; // 125Hz
  } else if (i <= 5) {
    band = 1; // 250Hz
  } else if (i <= 7) {
    band = 2; // 500Hz
  } else if (i <= 15) {
    band = 3; // 1000Hz
  } else if (i <= 30) {
    band = 4; // 2000Hz
  } else if (i <= 53) {
    band = 5; // 4000Hz
  } else if (i <= 106) {
    band = 6; // 8000Hz
  } else {
    band = 7;
  }
  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band]) {
    peak[band] = dsize;
  }
}

void renderFFT(void *parameter) {
  int item = 0;
  for (;;) {
    if (uxQueueMessagesWaiting(queue) > 0) {

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

      for (uint8_t band = 0; band < NUM_BANDS; band++) {
        peak[band] = 0;
      }

      // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
      for (int i = 2; i < (SAMPLES / 2); i++) {
        if (vReal[i] > 2000) { // Add a crude noise filter, 10 x amplitude or more
          createBands(i, (int)vReal[i] / amplitude);
        }
      }

      // Release handle
      xQueueReceive(queue, &item, 0);

      uint8_t intensity;
      FastLED.clear();
      FastLED.setBrightness(BRIGHTNESS);
      
      if(PATTERN == 0) {
          for (byte band = 0; band < NUM_BANDS; band++) {
          int bandLeds = LED_COUNT / NUM_BANDS;
          intensity = map(peak[band], 1, amplitude, 0, bandLeds);
            for (int i = 0; i < bandLeds; i++) {
              leds[(bandLeds * band) + i] = (i >= intensity) ? CHSV(0, 0, 0) : CHSV((band * 16), 255, 255);
            }
          }
        }
        else if(PATTERN == 1) {
          byte foundBand = -1; 
          for(byte band = NUM_BANDS - 1; band >= 0; band--) {
            intensity = map(peak[band], 1, amplitude, 0, 8);
            if(intensity > 0) {
              foundBand = band;
              break;
            }
          }
          if(foundBand > -1) {
            for (int i = 0; i < LED_COUNT; i++) {
              leds[i] = CHSV((foundBand * 16) , 255, 255);
            }
          }
        }
      FastLED.show();
    }
  }
}

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, DATA_PIN>(leds, LED_COUNT);
  queue = xQueueCreate(1, sizeof(int));
  if (queue == NULL) {
    Serial.println("Error creating the A2DP->FFT queue");
  }
  xTaskCreatePinnedToCore(renderFFT,      // Function that should be called
                          "FFT Renderer", // Name of the task (for debugging)
                          10000,          // Stack size (bytes)
                          NULL,           // Parameter to pass
                          1,              // Task priority
                          NULL,           // Task handle
                          1               // Core you want to run the task on (0 or 1)
  );
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start((char *)DEVICE_NAME);
  a2dp_sink.set_stream_reader(audio_data_callback);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
//  pinMode(BRIGHTNESS_BUTTON, INPUT_PULLUP);
//  pinMode(PATTERN_BUTTON, INPUT_PULLUP);
  Serial.println("connected");
}    

void loop() {
//  brightnessButtonCurrentState = digitalRead(BRIGHTNESS_BUTTON);
//  if(brightnessButtonState == HIGH && brightnessButtonCurrentState == LOW) {
//    BRIGHTNESS = (BRIGHTNESS + 10) % 100;
//    if(BRIGHTNESS == 0) {
//      BRIGHTNESS = 5;
//    }
//    delay(1000);
//  }
//  brightnessButtonState = brightnessButtonCurrentState;
  
//  patternButtonCurrentState = digitalRead(PATTERN_BUTTON);
//  if(patternButtonState == HIGH && patternButtonCurrentState == LOW) {
//    PATTERN = (PATTERN + 1) % TOTAL_PATTERN;
//    Serial.println("PATTERN");
//    Serial.println(PATTERN);
//    delay(1000);
//  }
//  patternButtonState = patternButtonCurrentState;
}
