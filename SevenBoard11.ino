#include <Arduino.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <driver/adc.h>

#include "fft.h"
#include "fft.c"
#include "debounce.h"

const String SYSTEM_VERSION = "1.1";

const char* wifiSSID = "SEVENBOARD1";
const char* wifiPassword = "grillaj0";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
 
#define PIN_SW1 35
#define PIN_SW2 39

#define SAMPLES_SIZE 512

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define SAMPLES_PER_LINE (SAMPLES_SIZE / DISPLAY_WIDTH)

#define N_DISPLAY_MODES 3
#define DISPLAY_WAVE 1
#define DISPLAY_FFT 2
#define DISPLAY_OUTPUTS 3

#define N_INPUT_CHANNELS 2
#define N_OUTPUT_CHANNELS 5

#define OUTPUT_CUTOFF 10
#define INPUT_BOOST 2

#define CHAN_MODE_STATIC 0
#define CHAN_MODE_FFT 1
#define CHAN_HISTORY_SIZE 4

struct input_channel_t {
  adc1_channel_t  chan;
  adc_atten_t     atten;
  int16_t         zeroCenter;
};

struct input_channel_t DRAM_ATTR inputChannels[N_INPUT_CHANNELS] = {
  { ADC1_CHANNEL_0, ADC_ATTEN_11db, 2048 }, // 3.5mm
  { ADC1_CHANNEL_6, ADC_ATTEN_6db, 2700 },  // mic module
};

uint8_t currentInputIndex = 1;
struct input_channel_t DRAM_ATTR *currentInput = &inputChannels[currentInputIndex];

struct disp_line_t {
  int8_t y1;
  int8_t y2;
};

bool blinkOn = false;
uint8_t displayMode = DISPLAY_FFT;
bool configMode = false;
struct disp_line_t dispLines[DISPLAY_WIDTH];
uint32_t frameCount = 0;
uint32_t startMillis;

struct output_channel_pins_t {
  int16_t pin_r;
  int16_t pin_g;
  int16_t pin_b;
  int16_t chan_r;
  int16_t chan_g;
  int16_t chan_b;
  uint8_t history_r[CHAN_HISTORY_SIZE];
  uint8_t history_g[CHAN_HISTORY_SIZE];
  uint8_t history_b[CHAN_HISTORY_SIZE];
  int16_t history_last_pos;
};

struct output_channel_pins_t outputChannelPins[N_OUTPUT_CHANNELS] = {
  { pin_r: 33, pin_g: 32, pin_b: 25 },
  { pin_r: 27, pin_g: 26, pin_b: 13 },
  { pin_r: 18, pin_g: 19, pin_b: 5 },
  { pin_r: 16, pin_g: 17, pin_b: 4 },
  { pin_r: 15, pin_g: 14, pin_b: 23 },
};

struct output_channel_config_t {
  uint8_t chan_mode;
  uint8_t fslot_r;
  uint8_t fslot_g;
  uint8_t fslot_b;
  bool showLines;
  uint8_t fwidth_r;
  uint8_t fwidth_g;
  uint8_t fwidth_b;
  uint8_t static_r;
  uint8_t static_g;
  uint8_t static_b;
};

struct output_channel_config_t outputChannelConfigs[N_OUTPUT_CHANNELS] = {
  { chan_mode: CHAN_MODE_FFT, fslot_r: 6, fslot_g: 45, fslot_b: 60, showLines: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT, fslot_r: 6, fslot_g: 45, fslot_b: 60, showLines: false, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT, fslot_r: 6, fslot_g: 45, fslot_b: 60, showLines: false, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT, fslot_r: 6, fslot_g: 45, fslot_b: 60, showLines: false, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT, fslot_r: 6, fslot_g: 45, fslot_b: 60, showLines: false, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
};

uint32_t lastBlink = 0;
DebounceButton btnConfig(PIN_SW1);
DebounceButton btnSwitch(PIN_SW2);

// Used by timer ISR
int16_t DRAM_ATTR abuf[SAMPLES_SIZE];
int16_t DRAM_ATTR abuf2[SAMPLES_SIZE];
int16_t volatile DRAM_ATTR abufPos = 0;
volatile bool DRAM_ATTR abuf2Ready = false;

int16_t fftBuf[DISPLAY_WIDTH];
volatile bool dbufReady = false;
volatile uint32_t dspCount = 0;
volatile uint32_t oldDspCount = 0; // used to detect transition into new dsp result in loop()
volatile uint32_t loopCount = 0;
volatile uint32_t outputUpdateCount = 0;
uint32_t oldFreeHeap = 0;
bool spiffsAvailable = false;
bool displayAvailable = false;

fft_config_t *fftc;

hw_timer_t * adcTimer = NULL; // our timer
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t dspTaskHandle;
TaskHandle_t adcTaskHandle;


// ============================================================================================================================== onTimer()
void IRAM_ATTR onTimer() {
  // Fills abuf with ADC readings. The abuf is FFT-sized (e.g. 512 samples), and when it gets full,
  // the adcTask is notified to take over.
  portENTER_CRITICAL_ISR(&timerMux);

  abuf[abufPos++] = adc1_get_voltage(currentInput->chan);
  
  if (abufPos >= SAMPLES_SIZE) { 
    abufPos = 0;

    // Notify adcTask that the buffer is full.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(adcTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ============================================================================================================================== adcTask()
void adcTask(void *param) {
  // Probably overkill: double-buffers abuf into abuf2 and notifies dspTask to analyse it.
  // Can probably be merged with dspTask, except maybe the difference in ulTaskNotify(pdFALSE...)
  while (true) {

    uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));
    // We got a signal that abuf is full
    
    if (!abuf2Ready) {
      memcpy(abuf2, abuf, sizeof(abuf2)); 
      abuf2Ready = true;
      xTaskNotify(dspTaskHandle, 0, eIncrement);
    }
  }
}

// ============================================================================================================================== dspTask()
void dspTask(void *param) {
  // Analyses ADC data read by the onTimer ISR, creates FFT and dispLines outputs

  while (true) {
    uint32_t tval = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    if (abuf2Ready) {
      // FFT is always performed
      for (int i = 0; i < SAMPLES_SIZE; i++) {
        fftc->input[i] = (float)(currentInput->zeroCenter - abuf2[i]);
      }
      fft_execute(fftc);
      for (int i = 0; i < DISPLAY_WIDTH; i++) {
        float ssum = 0;
        for (int j = (i * SAMPLES_PER_LINE); j < ( (i+1) * SAMPLES_PER_LINE ); j++) {
          if (j % 2 == 1) {
            ssum += abs(fftc->output[j]);
          }
        }
        fftBuf[i] = (ssum / SAMPLES_PER_LINE) / 300;
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
      updateOutputs();
    }
  }
}

// ============================================================================================================================== fromRawValue()
inline int16_t fromRawValue(int16_t x) {
  if (x > OUTPUT_CUTOFF) {
    return x * INPUT_BOOST;
  }
  return 0;
}

// ============================================================================================================================== getRedOutput()
int16_t getRedOutput(int16_t i) {
  int32_t count = 0;
  int32_t sum = 0;
  for (int16_t j = max(0, outputChannelConfigs[i].fslot_r - outputChannelConfigs[i].fwidth_r); j <= min(outputChannelConfigs[i].fslot_r + outputChannelConfigs[i].fwidth_r, DISPLAY_WIDTH-1); j++) {
    sum += fromRawValue(fftBuf[j]);
    count++;
  }
  int32_t v = sum / count;
  return (v*v)/16;
}


// ============================================================================================================================== getGreenOutput()
int16_t getGreenOutput(int16_t i) {
  int32_t count = 0;
  int32_t sum = 0;
  for (int16_t j = max(0, outputChannelConfigs[i].fslot_g - outputChannelConfigs[i].fwidth_g); j <= min(outputChannelConfigs[i].fslot_g + outputChannelConfigs[i].fwidth_g, DISPLAY_WIDTH-1); j++) {
    sum += fromRawValue(fftBuf[j]);
    count++;
  }
  int32_t v = sum / count;
  return (v*v)/16;
}


// ============================================================================================================================== getBlueOutput()
int16_t getBlueOutput(int16_t i) {
  int32_t count = 0;
  int32_t sum = 0;
  for (int16_t j = max(0, outputChannelConfigs[i].fslot_b - outputChannelConfigs[i].fwidth_b); j <= min(outputChannelConfigs[i].fslot_b + outputChannelConfigs[i].fwidth_b, DISPLAY_WIDTH-1); j++) {
    sum += fromRawValue(fftBuf[j]);
    count++;
  }
  int32_t v = sum / count;
  return (v*v)/16;
}

// ============================================================================================================================== drawDisplay()
void drawDisplay() {
  if (!displayAvailable)
    return;
  // Unfortunately, sending data to the I2C display from the DSP task results in a corrupted display :(
  // so we need to do it from loop()
  u8g2.clearBuffer();

  if (configMode) {
    u8g2.drawStr(0, 10, "WiFi config enabled");
    u8g2.drawStr(0, 20, wifiSSID);
    u8g2.drawStr(0, 30, wifiPassword);
    u8g2.sendBuffer();
    frameCount++;
    return;
  }

  if (displayMode == DISPLAY_FFT || displayMode == DISPLAY_WAVE) {
    for (int i = 0; i < DISPLAY_WIDTH; i++) {
      u8g2.drawLine(i, dispLines[i].y1, i, dispLines[i].y2);
    }
    //u8g2.setDrawColor(2);
    if (displayMode == DISPLAY_FFT) {
      for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
        if (!outputChannelConfigs[i].showLines) {
          continue;
        }
        u8g2.drawVLine(outputChannelConfigs[i].fslot_r, 0, DISPLAY_HEIGHT-1);
        u8g2.drawVLine(outputChannelConfigs[i].fslot_g, 0, DISPLAY_HEIGHT-1);
        u8g2.drawVLine(outputChannelConfigs[i].fslot_b, 0, DISPLAY_HEIGHT-1);
      }
      u8g2.drawHLine(0, DISPLAY_HEIGHT-1-OUTPUT_CUTOFF, DISPLAY_WIDTH);
    }
  } else if (displayMode == DISPLAY_OUTPUTS) {
    const int bar_width = DISPLAY_WIDTH / (N_OUTPUT_CHANNELS * 4) - 1;

    int col = 0;
    for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
      // output channel pins go from 0-255, so divide by 4 to fit into screen
      uint8_t red = outputChannelPins[i].history_r[outputChannelPins[i].history_last_pos] / 4;
      uint8_t green = outputChannelPins[i].history_g[outputChannelPins[i].history_last_pos] / 4;
      uint8_t blue = outputChannelPins[i].history_b[outputChannelPins[i].history_last_pos] / 4;

      for (int j = 0; j < bar_width; j++) {
        u8g2.drawVLine(col++, DISPLAY_HEIGHT-1-(red/4), DISPLAY_HEIGHT-1);
      }
      col += bar_width / 2;

      for (int j = 0; j < bar_width; j++) {
        u8g2.drawVLine(col++, DISPLAY_HEIGHT-1-(green/4), DISPLAY_HEIGHT-1);
      }
      col += bar_width / 2;

      for (int j = 0; j < bar_width; j++) {
        u8g2.drawVLine(col++, DISPLAY_HEIGHT-1-(blue/4), DISPLAY_HEIGHT-1);
      }
      col += bar_width / 2 + 2;
      
    }
  }
    
  //u8g2.setDrawColor(1);
  if (millis() - startMillis > 1000) {
    char buf[30];
    int seconds = ((millis() - startMillis) / 1000);
    
    sprintf(buf, "F %02d, A %d, O %d, I %d", frameCount / seconds, dspCount / seconds, outputUpdateCount / seconds, currentInputIndex);
    u8g2.drawStr(0, 9, buf);
  }
  
  u8g2.sendBuffer();
  frameCount++;
}

// ============================================================================================================================== updateOutputs()
void updateOutputs() {
  outputUpdateCount++;
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {

    outputChannelPins[i].history_last_pos++;
    if (outputChannelPins[i].history_last_pos >= CHAN_HISTORY_SIZE)
      outputChannelPins[i].history_last_pos = 0;

    int16_t red = outputChannelPins[i].history_r[outputChannelPins[i].history_last_pos] = getRedOutput(i);
    int16_t green = outputChannelPins[i].history_g[outputChannelPins[i].history_last_pos] = getGreenOutput(i);
    int16_t blue = outputChannelPins[i].history_b[outputChannelPins[i].history_last_pos] = getBlueOutput(i);
    
    ledcWrite(outputChannelPins[i].chan_r, red);
    ledcWrite(outputChannelPins[i].chan_g, green);
    ledcWrite(outputChannelPins[i].chan_b, blue);
  }
}

// ============================================================================================================================== handleButtons()
void handleButtons() {
  btnConfig.update();
  btnSwitch.update();
  
  if (btnConfig.check_depressed() > 0) {
    Serial.println("btnConfig depressed");
    if (!configMode) {
      configMode = true;
      timerAlarmDisable(adcTimer);
      delay(100);
      setupWiFi();
    } else {
      disableWiFi();
      delay(100);
      timerAlarmEnable(adcTimer);
      configMode = false;
    }
  }

  if (btnSwitch.pressed_for(1000)) {
    Serial.println("btnSwitch long-pressed");
    currentInputIndex++;
    if (currentInputIndex >= N_INPUT_CHANNELS) {
      currentInputIndex = 0;
    }
    currentInput = &inputChannels[currentInputIndex];
  } 
  
  if (btnSwitch.check_depressed() > 0) {
    Serial.println("btnSwitch short-depressed");
    switch (displayMode) {
      case DISPLAY_WAVE:
        displayMode = DISPLAY_FFT;
        break;
      case DISPLAY_FFT:
        displayMode = DISPLAY_OUTPUTS;
        break;
      case DISPLAY_OUTPUTS:
        displayMode = DISPLAY_WAVE;
        break;
    }    
  }
}

// ============================================================================================================================== setup()
void setup() {
  Serial.begin(9600);
  Serial.println("Starting setup");
  Serial.print("System version: ");
  Serial.print(SYSTEM_VERSION);
  Serial.print(", chip version: ");
  Serial.print(ESP.getChipRevision());
  Serial.print(", CPU frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print(" MHz, firmware signature: ");
  Serial.println(ESP.getSketchMD5());
  
  Wire.setClock(400000);
  
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    spiffsAvailable = false;
  } else {
    spiffsAvailable = true;
  }

  fftc = fft_init(SAMPLES_SIZE, FFT_REAL, FFT_FORWARD, NULL, NULL);
  
  if (u8g2.begin()) {
    u8g2.setFont(u8g2_font_t0_11b_mr );
    //u8g2.setFontMode(1);
    u8g2.clearBuffer();
    displayAvailable = true;
  }

  pinMode(2, OUTPUT);
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);

  int pwm_chan = 0;
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
    pinMode(outputChannelPins[i].pin_r, OUTPUT);
    ledcSetup(pwm_chan, 2000, 8);
    ledcAttachPin(outputChannelPins[i].pin_r, pwm_chan);
    outputChannelPins[i].chan_r = pwm_chan;
    pwm_chan++;

    pinMode(outputChannelPins[i].pin_g, OUTPUT);
    ledcSetup(pwm_chan, 2000, 8);
    ledcAttachPin(outputChannelPins[i].pin_g, pwm_chan);
    outputChannelPins[i].chan_g = pwm_chan;
    pwm_chan++;

    pinMode(outputChannelPins[i].pin_b, OUTPUT);
    ledcSetup(pwm_chan, 2000, 8);
    ledcAttachPin(outputChannelPins[i].pin_b, pwm_chan);
    outputChannelPins[i].chan_b = pwm_chan;
    pwm_chan++;    
  }

  adc1_config_width(ADC_WIDTH_12Bit);
  for (int i = 0; i < N_INPUT_CHANNELS; i++) {
    adc1_config_channel_atten(inputChannels[i].chan, inputChannels[i].atten);
  }
  
  //setupWiFi();
  loadOutputChannels();

  Serial.println("1");
  xTaskCreate(dspTask, "DSP Task", 8192, NULL, 1, &dspTaskHandle);
  Serial.println("2");
  xTaskCreate(adcTask, "ADC Task", 8192, NULL, 1, &adcTaskHandle);

  Serial.println("3");
  adcTimer = timerBegin(3, 80, true); // 80 Prescaler
  timerAttachInterrupt(adcTimer, &onTimer, true); // binds the handling function to our timer 
  timerAlarmWrite(adcTimer, 45, true);
  timerAlarmEnable(adcTimer);
  Serial.println("4");

  startMillis = lastBlink = millis();
  Serial.println("setup over");
}


// ============================================================================================================================== loop()
void loop() {
  webserverTasks();
  
  if (millis() - lastBlink > 1000) {
    blinkOn = !blinkOn;
    digitalWrite(2, blinkOn ? HIGH : LOW);
    lastBlink = millis();

    if (ESP.getFreeHeap() != oldFreeHeap) {
      Serial.print("Memory available: ");
      Serial.print(ESP.getFreeHeap());
      Serial.println(" bytes.");
      oldFreeHeap = ESP.getFreeHeap();
    }
  }

  handleButtons();

  if (dbufReady) {
    drawDisplay();
    dbufReady = false;
  }

  loopCount++;
}
