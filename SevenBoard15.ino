/*
 * TODO:
 *     - fix Knockout binding problem with chan_mode
 *     
 * I2C addresses:
 *     - 0x3C : OLED
 *     - 0x45 : INA219
 *     - 0x48 : LM75B
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <FastLED.h>
#include <Adafruit_INA219.h>
#include <Temperature_LM75_Derived.h>

#include "fft.h"
#include "fft.c"
#include "debounce.h"

#define DEBUG

const String SYSTEM_VERSION = "1.5.0";

const char *wifiSSIDPrefix = "SEVENBOARD_";
const char *wifiPasswordPrefix = "grillaj";

String wifiSSID;
String wifiPassword;

#define OLED_I2C_ADDRESS 0x3c
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define INA219_I2C_ADDRESS 0x45
Adafruit_INA219 ina219(INA219_I2C_ADDRESS);

#define LM75_I2C_ADDRESS 0x48
Generic_LM75 lm75;

#define PIN_WS2812 12
#define N_WS2812_LEDS 1
CRGB ws2812[N_WS2812_LEDS];

#define PIN_SW1 35
#define PIN_SW2 39

#define ADC_SAMPLES_COUNT 512

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define SAMPLES_PER_LINE (ADC_SAMPLES_COUNT / DISPLAY_WIDTH)

#define N_DISPLAY_MODES 3
#define DISPLAY_WAVE 1
#define DISPLAY_FFT 2
#define DISPLAY_OUTPUTS 3

#define N_INPUT_CHANNELS 2
#define N_OUTPUT_CHANNELS 5

#define CHAN_MODE_STATIC 0
#define CHAN_MODE_FFT_RGB 1
#define CHAN_MODE_FFT_RGBL 2

#define CHAN_HISTORY_SIZE 4

#define FSLOT_OP_AVG 0
#define FSLOT_OP_MIN 1
#define FSLOT_OP_MAX 2
#define FSLOT_OP_GEO 3

struct input_channel_t {
  adc1_channel_t  chan;
  adc_atten_t     atten;
  int16_t         zeroCenter;
  int16_t         dispZeroCenter;
};

struct input_channel_t DRAM_ATTR inputChannels[N_INPUT_CHANNELS] = {
  // atten: 0, 6 (3700), 11 (1850)
  { ADC1_CHANNEL_0, ADC_ATTEN_0db, 2048 }, // 3.5mm
  { ADC1_CHANNEL_6, ADC_ATTEN_6db, 2048, -20 },  // mic module
};

uint8_t currentInputIndex = 1;
struct input_channel_t DRAM_ATTR *currentInput = &inputChannels[currentInputIndex];

struct disp_line_t {
  int8_t y1;
  int8_t y2;
};

bool blinkOn = false;
bool webServerSetup = false;
uint8_t displayMode = DISPLAY_FFT;
bool configMode = false;
String configModeMessage = "";
uint32_t configModeMessageTime = 0;
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
  int16_t last_r;
  int16_t last_g;
  int16_t last_b;
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
  uint8_t fslot_op;
  bool show_lines;
  bool history_filter;
  uint8_t fwidth_r;
  uint8_t fwidth_g;
  uint8_t fwidth_b;
  uint8_t static_r;
  uint8_t static_g;
  uint8_t static_b;
  uint8_t rgbl_r;
  uint8_t rgbl_g;
  uint8_t rgbl_b;
  uint8_t rgbl_fslot;
  uint8_t rgbl_fwidth;
  uint16_t fslot_damp;
};

struct output_channel_config_t outputChannelConfigs[N_OUTPUT_CHANNELS] = {
  { chan_mode: CHAN_MODE_FFT_RGB, fslot_r: 6, fslot_g: 45, fslot_b: 60, fslot_op: FSLOT_OP_AVG, show_lines: true,  history_filter: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT_RGB, fslot_r: 6, fslot_g: 45, fslot_b: 60, fslot_op: FSLOT_OP_AVG, show_lines: false, history_filter: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT_RGB, fslot_r: 6, fslot_g: 45, fslot_b: 60, fslot_op: FSLOT_OP_AVG, show_lines: false, history_filter: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT_RGB, fslot_r: 6, fslot_g: 45, fslot_b: 60, fslot_op: FSLOT_OP_AVG, show_lines: false, history_filter: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
  { chan_mode: CHAN_MODE_FFT_RGB, fslot_r: 6, fslot_g: 45, fslot_b: 60, fslot_op: FSLOT_OP_AVG, show_lines: false, history_filter: true, fwidth_r: 2, fwidth_g: 5, fwidth_b: 10 },
};

uint32_t lastBlink = 0;
DebounceButton btnConfig(PIN_SW1);
DebounceButton btnSwitch(PIN_SW2);

// Used by timer ISR
int16_t DRAM_ATTR abuf[ADC_SAMPLES_COUNT];
int16_t DRAM_ATTR abuf2[ADC_SAMPLES_COUNT];
int16_t volatile DRAM_ATTR abufPos = 0;
volatile bool DRAM_ATTR abuf2Ready = false;

int16_t fftSlot[DISPLAY_WIDTH];
volatile bool dbufReady = false;
volatile uint32_t dspCount = 0;
volatile uint32_t oldDspCount = 0; // used to detect transition into new dsp result in loop()
volatile uint32_t loopCount = 0;
volatile uint32_t outputUpdateCount = 0;
uint32_t oldFreeHeap = 0;
bool spiffsAvailable = false;
bool displayAvailable = false;
bool ina219Available = false;
bool lm75Available = false;

uint32_t inputBoost = 5;
uint32_t outputDampen = 32;
uint32_t outputCutoff = 3;

fft_config_t *fftc;

hw_timer_t * adcTimer = NULL; // our timer
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t dspTaskHandle;
TaskHandle_t adcTaskHandle;


// ============================================================================================================================== local_adc1_read()
// Adapted from the ESP32 IDF SDK's adc_convert() so that it's fixed in IRAM
int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); //only one channel is selected.
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

// ============================================================================================================================== onTimer()
void IRAM_ATTR onTimer() {
  // Fills abuf with ADC readings. The abuf is FFT-sized (e.g. 512 samples), and when it gets full,
  // the adcTask is notified to take over.
  portENTER_CRITICAL_ISR(&timerMux);

  //abuf[abufPos++] = adc1_get_voltage(currentInput->chan);
  abuf[abufPos++] = local_adc1_read(currentInput->chan);
  
  if (abufPos >= ADC_SAMPLES_COUNT) { 
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
  float fInputBoost = ((float)inputBoost)/5.0;

  while (true) {
    uint32_t tval = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    if (abuf2Ready) {
      // FFT is always performed
      for (int i = 0; i < ADC_SAMPLES_COUNT; i++) {
        fftc->input[i] = (float)(currentInput->zeroCenter - abuf2[i]);
      }
      fft_execute(fftc);
      for (int i = 0; i < DISPLAY_WIDTH; i++) {
        float ssum = 0;
        for (int j = (i * SAMPLES_PER_LINE); j < ( (i+1) * SAMPLES_PER_LINE ); j++) {
          if (j % 2 == 1) {
            ssum += abs(fftc->output[j] * fInputBoost);
          }
        }
        fftSlot[i] = (ssum / SAMPLES_PER_LINE) / 300;
      }

/*      for (int i = 0; i < fftSlot[1] / 2; i++) {
          Fire2012();
      }*/

      // Draw something into dispLines, depending on the setting
      if (displayMode == DISPLAY_WAVE) {  
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
          int ssum = 0;
          for (int j = (i * SAMPLES_PER_LINE); j < ( (i+1) * SAMPLES_PER_LINE ); j++) {
            ssum += (currentInput->zeroCenter - abuf2[j]);
          }
          
          int amp = (ssum / SAMPLES_PER_LINE) / 75  - currentInput->dispZeroCenter;
          dispLines[i].y1 = 31;
          dispLines[i].y2 = 31 + amp;
        }
      } else if (displayMode == DISPLAY_FFT) {
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
          dispLines[i].y1 = 63;
          dispLines[i].y2 = 63 - fftSlot[i];
        }
      }
      dbufReady = true;
      abuf2Ready = false;

      dspCount++;
      updateOutputs();
    }
  }
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
    u8g2.drawStr(0, 20, wifiSSID.c_str());
    u8g2.drawStr(0, 30, wifiPassword.c_str());

    if (configModeMessage != String("")) {
      u8g2.drawStr(0, 60, configModeMessage.c_str());
      if (millis() - configModeMessageTime > 2000) {
        configModeMessage = "";
      }
    }
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
        if (!outputChannelConfigs[i].show_lines) {
          continue;
        }
        if (outputChannelConfigs[i].chan_mode == CHAN_MODE_FFT_RGB) {
          u8g2.drawVLine(outputChannelConfigs[i].fslot_r, 0, DISPLAY_HEIGHT-1);
          u8g2.drawVLine(outputChannelConfigs[i].fslot_g, 0, DISPLAY_HEIGHT-1);
          u8g2.drawVLine(outputChannelConfigs[i].fslot_b, 0, DISPLAY_HEIGHT-1);
        } else if (outputChannelConfigs[i].chan_mode == CHAN_MODE_FFT_RGBL) {
          u8g2.drawVLine(outputChannelConfigs[i].rgbl_fslot, 0, DISPLAY_HEIGHT-1);
        }
      }
      u8g2.drawHLine(0, DISPLAY_HEIGHT-1-outputCutoff, DISPLAY_WIDTH);
    }
  } else if (displayMode == DISPLAY_OUTPUTS) {
    const int bar_width = DISPLAY_WIDTH / (N_OUTPUT_CHANNELS * 4) - 1;

    int col = 0;
    for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
      uint8_t red = outputChannelPins[i].last_r / 4;
      uint8_t green = outputChannelPins[i].last_g / 4;
      uint8_t blue = outputChannelPins[i].last_b / 4;

      for (int j = 0; j < bar_width; j++) {
        u8g2.drawVLine(col++, DISPLAY_HEIGHT-1-(red), DISPLAY_HEIGHT-1);
      }
      col += bar_width / 2;

      for (int j = 0; j < bar_width; j++) {
        u8g2.drawVLine(col++, DISPLAY_HEIGHT-1-(green), DISPLAY_HEIGHT-1);
      }
      col += bar_width / 2;

      for (int j = 0; j < bar_width; j++) {
        u8g2.drawVLine(col++, DISPLAY_HEIGHT-1-(blue), DISPLAY_HEIGHT-1);
      }
      col += bar_width / 2 + 3;
    }
  }
    
#ifdef DEBUG
  //u8g2.setDrawColor(1);
  if (millis() - startMillis > 1000 && displayMode == DISPLAY_FFT) {
    char buf[30];
    int seconds = ((millis() - startMillis) / 1000);
    
    sprintf(buf, "F %02d, A %d, O %d, I %d", frameCount / seconds, dspCount / seconds, outputUpdateCount / seconds, currentInputIndex);
    u8g2.drawStr(0, 9, buf);
    sprintf(buf, "Up: %d m.", seconds/60);
    if (lm75Available) {
      char t[10];
      sprintf(t, " %0.1f C", lm75.readTemperatureC());
      strcat(buf, t);
    }
    if (webServerSetup) {
      strcat(buf, " [W]");
    }
    u8g2.drawStr(0, 18, buf);

    if (ina219Available) {
      sprintf(buf, "%3.1f V, %3.1f A", ina219.getBusVoltage_V(), ina219.getCurrent_mA());
      u8g2.drawStr(0, 27, buf);
    }
  }
#endif
  u8g2.sendBuffer();
  frameCount++;
}


// ============================================================================================================================== fromRawValue()
inline int32_t fromRawValue(int32_t x) {
  if (x > outputCutoff) {
    return int32_t(x) /** inputBoost*/;
  }
  return 0;
}

// ============================================================================================================================== getFFTslotOutput()
int16_t getFFTslotOutput(int16_t fslot, int16_t fwidth, int16_t fslot_op, uint32_t fslot_damp) {
  uint32_t v;
  int count = 0;
  int sum = 0;

  switch(fslot_op) {
    case FSLOT_OP_AVG:
      for (int16_t j = max(0, fslot - fwidth); j <= min(fslot + fwidth, DISPLAY_WIDTH-1); j++) {
        sum += fromRawValue(fftSlot[j]);
        count++;
      }
      v = sum / count;
      break;
    case FSLOT_OP_MIN:
      v = INT_MAX;
      for (int16_t j = max(0, fslot - fwidth); j <= min(fslot + fwidth, DISPLAY_WIDTH-1); j++) {
        int32_t vv = fromRawValue(fftSlot[j]);
        if (vv < v) {
          v = vv;
        }
      }
      break;
    case FSLOT_OP_MAX:
      v = 0;
      for (int16_t j = max(0, fslot - fwidth); j <= min(fslot + fwidth, DISPLAY_WIDTH-1); j++) {
        int32_t vv = fromRawValue(fftSlot[j]);
        if (vv > v) {
          v = vv;
        }
      }
      break;
    case FSLOT_OP_GEO:
      // Geometric mean
      sum = 0;
      bool first = true;
      for (int16_t j = max(0, fslot - fwidth); j <= min(fslot + fwidth, DISPLAY_WIDTH-1); j++) {
        int32_t vv = fromRawValue(fftSlot[j]);
        if (first) {
          v = vv;
          first = false;
        } else {
          v *= vv;
        }
        sum++;
      }
      v = uint32_t(pow(v, 1.0/sum));
      break;
      
  }
  
  return min((v*v)/(outputDampen + fslot_damp), (uint32_t)255);
}

// ============================================================================================================================== updateOutputs()
void updateOutputs() {
  outputUpdateCount++;
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {

    outputChannelPins[i].history_last_pos++;
    if (outputChannelPins[i].history_last_pos >= CHAN_HISTORY_SIZE)
      outputChannelPins[i].history_last_pos = 0;

    uint16_t red; 
    uint16_t green; 
    uint16_t blue; 

    if (outputChannelConfigs[i].chan_mode == CHAN_MODE_STATIC) {
      red = outputChannelPins[i].history_r[outputChannelPins[i].history_last_pos] = outputChannelConfigs[i].static_r;
      green = outputChannelPins[i].history_g[outputChannelPins[i].history_last_pos] = outputChannelConfigs[i].static_g;
      blue = outputChannelPins[i].history_b[outputChannelPins[i].history_last_pos] = outputChannelConfigs[i].static_b;
    } else if (outputChannelConfigs[i].chan_mode == CHAN_MODE_FFT_RGB) {
      red = outputChannelPins[i].history_r[outputChannelPins[i].history_last_pos] = getFFTslotOutput(outputChannelConfigs[i].fslot_r, outputChannelConfigs[i].fwidth_r, outputChannelConfigs[i].fslot_op, outputChannelConfigs[i].fslot_damp);
      green = outputChannelPins[i].history_g[outputChannelPins[i].history_last_pos] = getFFTslotOutput(outputChannelConfigs[i].fslot_g, outputChannelConfigs[i].fwidth_g, outputChannelConfigs[i].fslot_op, outputChannelConfigs[i].fslot_damp);
      blue = outputChannelPins[i].history_b[outputChannelPins[i].history_last_pos] = getFFTslotOutput(outputChannelConfigs[i].fslot_b, outputChannelConfigs[i].fwidth_b, outputChannelConfigs[i].fslot_op, outputChannelConfigs[i].fslot_damp);
    } else if (outputChannelConfigs[i].chan_mode == CHAN_MODE_FFT_RGBL) {      
      CRGB c2(outputChannelConfigs[i].rgbl_r, outputChannelConfigs[i].rgbl_g, outputChannelConfigs[i].rgbl_b);
      c2.fadeToBlackBy(255 - getFFTslotOutput(outputChannelConfigs[i].rgbl_fslot, outputChannelConfigs[i].rgbl_fwidth, outputChannelConfigs[i].fslot_op, outputChannelConfigs[i].fslot_damp));
      
      red = outputChannelPins[i].history_r[outputChannelPins[i].history_last_pos] = c2.red;
      green = outputChannelPins[i].history_g[outputChannelPins[i].history_last_pos] = c2.green;
      blue = outputChannelPins[i].history_b[outputChannelPins[i].history_last_pos] = c2.blue;
    }

    if (outputChannelConfigs[i].history_filter) {
      red = sum_u8(outputChannelPins[i].history_r, CHAN_HISTORY_SIZE) / CHAN_HISTORY_SIZE;
      green = sum_u8(outputChannelPins[i].history_g, CHAN_HISTORY_SIZE) / CHAN_HISTORY_SIZE;
      blue = sum_u8(outputChannelPins[i].history_b, CHAN_HISTORY_SIZE) / CHAN_HISTORY_SIZE;
    }
    
    ledcWrite(outputChannelPins[i].chan_r, red);
    ledcWrite(outputChannelPins[i].chan_g, green);
    ledcWrite(outputChannelPins[i].chan_b, blue);

    outputChannelPins[i].last_r = red;
    outputChannelPins[i].last_g = green;
    outputChannelPins[i].last_b = blue;
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

  uint8_t esp32RawMac[6];
  esp_read_mac(esp32RawMac, ESP_MAC_WIFI_SOFTAP);
  String b32Mac = b32str(esp32RawMac, 6, false);
  wifiSSID = String((char*)wifiSSIDPrefix) + b32Mac.substring(b32Mac.length()-3);
  Serial.print("WiFi SSID: ");
  Serial.println(wifiSSID);
  wifiPassword = String((char*)wifiPasswordPrefix) + b32Mac.substring(b32Mac.length()-6, b32Mac.length()-4);
  Serial.print("WiFi password: ");
  Serial.println(wifiPassword);

  Wire.begin();
  Wire.setClock(100000);
  scanI2C();
  
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    spiffsAvailable = false;
  } else {
    spiffsAvailable = true;
  }

  fftc = fft_init(ADC_SAMPLES_COUNT, FFT_REAL, FFT_FORWARD, NULL, NULL);

  displayAvailable = isI2Cdevice(OLED_I2C_ADDRESS);
  if (displayAvailable) {
    u8g2.setBusClock(800000);  
    u8g2.begin();
    u8g2.setFont(u8g2_font_t0_11b_mr);
    //u8g2.setFontMode(1);
    u8g2.clearBuffer();
  }

  Wire.setClock(100000);
  ina219Available = isI2Cdevice(INA219_I2C_ADDRESS);
  if (ina219Available) {
    ina219.begin();
    ina219.setCalibration_32V_2A();
  }

  lm75Available = isI2Cdevice(LM75_I2C_ADDRESS);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
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

  loadChannelConfig();

  /*
  if (digitalRead(PIN_SW1) == LOW) {
    delay(500);
    if (digitalRead(PIN_SW1) == LOW) {
      setupWiFi();
      if (displayAvailable) {
        u8g2.drawStr(0, 30, "Turning WiFi on...");
        u8g2.sendBuffer();
      }
      digitalWrite(2, HIGH);
      delay(2000);
      digitalWrite(2, LOW);
      delay(1000);
    }
  }*/
  if (digitalRead(PIN_SW1) != LOW) {
    if (displayAvailable) {
      u8g2.drawStr(0, 30, "Turning WiFi on...");
      u8g2.sendBuffer();
    }
    setupWiFi();
  }

  adc1_config_width(ADC_WIDTH_12Bit);
  for (int i = 0; i < N_INPUT_CHANNELS; i++) {
    adc1_config_channel_atten(inputChannels[i].chan, inputChannels[i].atten);
    adc1_get_raw(inputChannels[i].chan); // prime the ADC
  }

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

  pinMode(PIN_WS2812, OUTPUT);
  FastLED.addLeds<WS2812B, PIN_WS2812, GRB>(ws2812, N_WS2812_LEDS);
  FastLED.setBrightness(200);

  startMillis = lastBlink = millis();
  Serial.println("setup over");
}


// ============================================================================================================================== loop()
void loop() {
  webserverTasks();
  
  if (millis() - lastBlink > (!configMode ? 1000 : 200)) {
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

  //FastLED.show();

  loopCount++;
}
