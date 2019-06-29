#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <driver/adc.h>

#include "fft.h"
#include "fft.c"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
 
#define DMIC_INPUT_PIN 34
#define SW1_PIN 12
#define SW2_PIN 14

#define SAMPLES_SIZE 512

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define SAMPLES_PER_LINE (SAMPLES_SIZE / DISPLAY_WIDTH)
#define DISPLAY_WAVE 1
#define DISPLAY_FFT 2
#define N_INPUT_CHANNELS 3
#define N_OUTPUT_CHANNELS 5
#define OUTPUT_CUTOFF 10
#define OUTPUT_BOOST 2

struct input_channel_t {
  adc1_channel_t  chan;
  adc_atten_t     atten;
  int16_t         zeroCenter;
};

struct input_channel_t DRAM_ATTR inputChannels[N_INPUT_CHANNELS] = {
  { ADC1_CHANNEL_0, ADC_ATTEN_11db, 2048 },
  { ADC1_CHANNEL_3, ADC_ATTEN_11db, 2048 },
  { ADC1_CHANNEL_6, ADC_ATTEN_6db, 2700 },
};

struct input_channel_t DRAM_ATTR *currentInput = &inputChannels[2];
uint8_t currentInputIndex = 2;

struct disp_line_t {
  int8_t y1;
  int8_t y2;
};

bool blinkOn = false;
uint8_t displayMode = DISPLAY_FFT;
struct disp_line_t dispLines[DISPLAY_WIDTH];
uint32_t frameCount = 0;
uint32_t startMillis;

struct output_channel_pins_t {
  int8_t pin_r;
  int8_t pin_g;
  int8_t pin_b;
  int8_t chan_r;
  int8_t chan_g;
  int8_t chan_b;
};

struct output_channel_pins_t outputChannelPins[N_OUTPUT_CHANNELS] = {
  { pin_r: 33, pin_g: 32, pin_b: 25 },
  { pin_r: 27, pin_g: 26, pin_b: 13 },
  { pin_r: 18, pin_g: 19, pin_b: 5 },
  { pin_r: 16, pin_g: 17, pin_b: 4 },
  { pin_r: 15, pin_g: 2,  pin_b: 23 },
};

struct output_channel_freqs_t {
  uint16_t fslot_r;
  uint16_t fslot_g;
  uint16_t fslot_b;
  bool showLines;
  uint8_t fwidth_r;
  uint8_t fwidth_g;
  uint8_t fwidth_b;
};

struct output_channel_freqs_t outputChannelFreqs[N_OUTPUT_CHANNELS] = {
  { fslot_r: 8, fslot_g: 45, fslot_b: 60, showLines: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { fslot_r: 8, fslot_g: 45, fslot_b: 60, showLines: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { fslot_r: 8, fslot_g: 45, fslot_b: 60, showLines: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { fslot_r: 8, fslot_g: 45, fslot_b: 60, showLines: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { fslot_r: 8, fslot_g: 45, fslot_b: 60, showLines: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
};

uint32_t lastBlink = 0;
uint32_t sw1LastTime = 0;
uint32_t sw2LastTime = 0;

// Used by timer ISR
int16_t DRAM_ATTR abuf[SAMPLES_SIZE];
int16_t DRAM_ATTR abuf2[SAMPLES_SIZE];
int16_t volatile DRAM_ATTR abufPos = 0;
volatile bool DRAM_ATTR abuf2Ready = false;

int16_t fftBuf[DISPLAY_WIDTH];
volatile bool dbufReady = false;
volatile uint32_t dspCount = 0;
volatile uint32_t oldDspCount = 0; // used to detect transition into new dsp result in loop()

fft_config_t *fftc;

hw_timer_t * timer = NULL; // our timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t dspTaskHandle;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  int adcVal = adc1_get_voltage(currentInput->chan);
  abuf[abufPos++] = adcVal;
  
  if (abufPos >= SAMPLES_SIZE) { 
    abufPos = 0;
    if (!abuf2Ready) {
      memcpy(abuf2, abuf, sizeof(abuf2)); 
      abuf2Ready = true; 
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux); // says that we have run our critical code
}


void dspTask(void *param) {

  while (true) {
    vTaskDelay(1);

    if (abuf2Ready) {

      // FFT is always done
      for (int i = 0; i < SAMPLES_SIZE; i++) {
        fftc->input[i] = (float)(currentInput->zeroCenter - abuf2[i]);
      }
      fft_execute(fftc);
      for (int i = 0; i < DISPLAY_WIDTH; i++) {
        float ssum = 0;
        for (int j = (i * SAMPLES_PER_LINE); j < ( (i+1) * SAMPLES_PER_LINE ); j++) {
          if (j % 2 == 1) {
            ssum += fftc->output[j];
          }
        }
        fftBuf[i] = abs((ssum / SAMPLES_PER_LINE) / 100);
      }

      // Draw something into dispLines, depending on the setting
      if (displayMode == DISPLAY_WAVE) {  
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
          int ssum = 0;
          for (int j = (i * SAMPLES_PER_LINE); j < ( (i+1) * SAMPLES_PER_LINE ); j++) {
            ssum += (currentInput->zeroCenter - abuf2[j]);
          }
          
          int amp = (ssum / SAMPLES_PER_LINE) / 75;
          dispLines[i].y1 = 31;
          dispLines[i].y2 = 31 + amp;
        }
      } else if (displayMode == DISPLAY_FFT) {
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
          dispLines[i].y1 = 63;
          dispLines[i].y2 = 63 - fftBuf[i];
        }
      }
      dbufReady = true;
      abuf2Ready = false;

      dspCount++;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("starting setup");
  Wire.setClock(400000);

  fftc = fft_init(SAMPLES_SIZE, FFT_REAL, FFT_FORWARD, NULL, NULL);
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_t0_16b_mf);
  //u8g2.setFontMode(1);
  u8g2.clearBuffer();

  pinMode(DMIC_INPUT_PIN, INPUT);
  pinMode(2, OUTPUT);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);

  int pwm_chan = 0;
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
    pinMode(outputChannelPins[i].pin_r, OUTPUT);
    ledcSetup(pwm_chan, 5000, 8);
    ledcAttachPin(outputChannelPins[i].pin_r, pwm_chan);
    outputChannelPins[i].chan_r = pwm_chan;
    pwm_chan++;

    pinMode(outputChannelPins[i].pin_g, OUTPUT);
    ledcSetup(pwm_chan, 5000, 8);
    ledcAttachPin(outputChannelPins[i].pin_g, pwm_chan);
    outputChannelPins[i].chan_g = pwm_chan;
    pwm_chan++;

    pinMode(outputChannelPins[i].pin_b, OUTPUT);
    ledcSetup(pwm_chan, 5000, 8);
    ledcAttachPin(outputChannelPins[i].pin_b, pwm_chan);
    outputChannelPins[i].chan_b = pwm_chan;
    pwm_chan++;    
  }

  adc1_config_width(ADC_WIDTH_12Bit);
  for (int i = 0; i < N_INPUT_CHANNELS; i++) {
    adc1_config_channel_atten(inputChannels[i].chan, inputChannels[i].atten);
  }
  
  setupWiFi();
  
  timer = timerBegin(3, 80, true); // 80 Prescaler
  timerAttachInterrupt(timer, &onTimer, true); // binds the handling function to our timer 
  timerAlarmWrite(timer, 45, true);
  timerAlarmEnable(timer);

  xTaskCreate(dspTask, "DSP Task", 8192, NULL, 1, &dspTaskHandle);

  startMillis = lastBlink = millis();
  Serial.println("setup over");
}

inline int16_t fromRawValue(int16_t x) {
  if (x > OUTPUT_CUTOFF) {
    return x * OUTPUT_BOOST;
  }
  return 0;
}

int16_t getRedOutput(int16_t i) {
  int16_t count = 0;
  int16_t sum = 0;
  for (int16_t j = max(0, outputChannelFreqs[i].fslot_r - outputChannelFreqs[i].fwidth_r); j < min(outputChannelFreqs[i].fslot_r + outputChannelFreqs[i].fwidth_r, DISPLAY_WIDTH-1); j++) {
    sum += fromRawValue(fftBuf[j]);
    count++;
  }
  return sum / count;
}


int16_t getGreenOutput(int16_t i) {
  int16_t count = 0;
  int16_t sum = 0;
  for (int16_t j = max(0, outputChannelFreqs[i].fslot_g - outputChannelFreqs[i].fwidth_g); j < min(outputChannelFreqs[i].fslot_g + outputChannelFreqs[i].fwidth_g, DISPLAY_WIDTH-1); j++) {
    sum += fromRawValue(fftBuf[j]);
    count++;
  }
  return sum / count;
}


int16_t getBlueOutput(int16_t i) {
  int16_t count = 0;
  int16_t sum = 0;
  for (int16_t j = max(0, outputChannelFreqs[i].fslot_b - outputChannelFreqs[i].fwidth_b); j < min(outputChannelFreqs[i].fslot_b + outputChannelFreqs[i].fwidth_b, DISPLAY_WIDTH-1); j++) {
    sum += fromRawValue(fftBuf[j]);
    count++;
  }
  return sum / count;
}


void loop() {
  webserverTasks();
  
  if (millis() - lastBlink > 1000) {
    blinkOn = !blinkOn;
    digitalWrite(2, blinkOn ? HIGH : LOW);
    lastBlink = millis();
  }

  if (digitalRead(SW1_PIN) == LOW && millis() - sw1LastTime > 500) {
    currentInputIndex++;
    if (currentInputIndex >= N_INPUT_CHANNELS) {
      currentInputIndex = 0;
    }
    currentInput = &inputChannels[currentInputIndex];
    sw1LastTime = millis();
  }

  if (digitalRead(SW2_PIN) == LOW && millis() - sw2LastTime > 500) {
    switch (displayMode) {
      case DISPLAY_WAVE:
        displayMode = DISPLAY_FFT;
        break;
      case DISPLAY_FFT:
        displayMode = DISPLAY_WAVE;
        break;
    }
    sw2LastTime = millis();
  }

  if (dbufReady) {
    // Unfortunately, sending data to the I2C display from the DSP task results in a corrupted display :(
    // so we need to do it from loop()
    u8g2.clearBuffer();
    for (int i = 0; i < DISPLAY_WIDTH; i++) {
      u8g2.drawLine(i, dispLines[i].y1, i, dispLines[i].y2);
    }
    //u8g2.setDrawColor(2);
    if (displayMode == DISPLAY_FFT) {
      for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
        if (!outputChannelFreqs[i].showLines) {
          continue;
        }
        u8g2.drawVLine(outputChannelFreqs[i].fslot_r, 0, DISPLAY_HEIGHT-1);
        u8g2.drawVLine(outputChannelFreqs[i].fslot_g, 0, DISPLAY_HEIGHT-1);
        u8g2.drawVLine(outputChannelFreqs[i].fslot_b, 0, DISPLAY_HEIGHT-1);
      }
      u8g2.drawHLine(0, DISPLAY_HEIGHT-1-OUTPUT_CUTOFF, DISPLAY_WIDTH);
    }
    //u8g2.setDrawColor(1);
    if (millis() - startMillis > 1000) {
      char buf[20];
      sprintf(buf, "%d FPS, %d DPS", frameCount / ((millis() - startMillis) / 1000), dspCount / ((millis() - startMillis) / 1000));
      u8g2.drawStr(0, 15, buf);
      sprintf(buf, "IN: %d", currentInputIndex);
      u8g2.drawStr(0, 30, buf);
    }
    
    u8g2.sendBuffer();
    dbufReady = false;
    frameCount++;
  }

  if (dspCount != oldDspCount) {
    for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
      ledcWrite(outputChannelPins[i].chan_r, getRedOutput(i));
      ledcWrite(outputChannelPins[i].chan_g, getGreenOutput(i));
      ledcWrite(outputChannelPins[i].chan_b, getBlueOutput(i));
    }
    oldDspCount = dspCount;
  }

}
