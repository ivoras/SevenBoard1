#include <SPIFFS.h>

#define OUTPUT_CHANNELS_SAVE_FILE "/output_channels.json"

void dumpOutputChannels() {
  Serial.println("Output channels:");
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
    char buf[80];
    sprintf(buf, "%d: R: %d\tRw: %d;\tG: %d\tGw: %d;\tB: %d\tBw: %d;\tLL: %d", i, 
      outputChannelConfigs[i].fslot_r, outputChannelConfigs[i].fwidth_r,
      outputChannelConfigs[i].fslot_g, outputChannelConfigs[i].fwidth_g,
      outputChannelConfigs[i].fslot_b, outputChannelConfigs[i].fwidth_b,
      outputChannelConfigs[i].show_lines);
    Serial.println(buf);
  }
}

void serializeOutputChannels(JsonObject &root) {
  JsonArray outputs = root.createNestedArray("outputs");
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
    JsonObject output = outputs.createNestedObject();
    output["i"] = i;

    output["chan_mode"] = outputChannelConfigs[i].chan_mode;
    output["show_lines"] = outputChannelConfigs[i].show_lines;
    output["history_filter"] = outputChannelConfigs[i].history_filter;
    
    output["fslot_r"] = outputChannelConfigs[i].fslot_r;
    output["fslot_g"] = outputChannelConfigs[i].fslot_g;
    output["fslot_b"] = outputChannelConfigs[i].fslot_b;
    
    output["fwidth_r"] = outputChannelConfigs[i].fwidth_r;
    output["fwidth_g"] = outputChannelConfigs[i].fwidth_g;
    output["fwidth_b"] = outputChannelConfigs[i].fwidth_b;

    output["static_r"] = outputChannelConfigs[i].static_r;
    output["static_g"] = outputChannelConfigs[i].static_g;
    output["static_b"] = outputChannelConfigs[i].static_b;

    output["hsv_h"] = outputChannelConfigs[i].hsv_h;
    output["hsv_s"] = outputChannelConfigs[i].hsv_s;
    output["hsv_v"] = outputChannelConfigs[i].hsv_v;
    output["hsv_freq"] = outputChannelConfigs[i].hsv_freq;
  }
}

void deserializeOutputChannels(JsonDocument &doc, struct output_channel_config_t channels[]) {
  
  JsonArray outputs = doc["outputs"];
  for (int n = 0; n < outputs.size(); n++) {
    int i = outputs[n]["i"];

    channels[i].chan_mode = outputs[n]["chan_mode"];
    channels[i].show_lines = outputs[n]["show_lines"];
    channels[i].history_filter = outputs[n]["history_filter"];

    channels[i].fslot_r = outputs[n]["fslot_r"];
    channels[i].fslot_g = outputs[n]["fslot_g"];
    channels[i].fslot_b = outputs[n]["fslot_b"];

    channels[i].fwidth_r = outputs[n]["fwidth_r"];
    channels[i].fwidth_g = outputs[n]["fwidth_g"];
    channels[i].fwidth_b = outputs[n]["fwidth_b"];

    channels[i].static_r = outputs[n]["static_r"];
    channels[i].static_g = outputs[n]["static_g"];
    channels[i].static_b = outputs[n]["static_b"];

    channels[i].hsv_h = outputs[n]["hsv_h"];
    channels[i].hsv_s = outputs[n]["hsv_s"];
    channels[i].hsv_v = outputs[n]["hsv_v"];
    channels[i].hsv_freq = outputs[n]["hsv_freq"];
  }
}

void saveChannelConfig() {
  timerStop(adcTimer);

  File f = SPIFFS.open(OUTPUT_CHANNELS_SAVE_FILE, FILE_WRITE);

  StaticJsonDocument<4096> jsonDoc;
  JsonObject root = jsonDoc.to<JsonObject>();
  
  root["sysVersion"] = SYSTEM_VERSION;
  root["inputBoost"] = inputBoost;
  root["outputDampen"] = outputDampen;
  root["outputCutoff"] = outputCutoff;
  serializeOutputChannels(root);
  
  serializeJson(jsonDoc, f);

  f.close();

  timerStart(adcTimer);
}

void loadChannelConfig() {
  bool timerActive = (adcTimer != NULL) && timerStarted(adcTimer);
  if (timerActive) {
    timerStop(adcTimer);
  }

  if (SPIFFS.exists(OUTPUT_CHANNELS_SAVE_FILE)) {
    File f = SPIFFS.open(OUTPUT_CHANNELS_SAVE_FILE, FILE_READ);
    StaticJsonDocument<4096> jsonDoc;
    auto error = deserializeJson(jsonDoc, f);
    if (!error) {
      String sysVersion = jsonDoc["sysVersion"];
      if (sysVersion == String(SYSTEM_VERSION)) {
        deserializeOutputChannels(jsonDoc, outputChannelConfigs);
        inputBoost = jsonDoc["inputBoost"];
        outputDampen = jsonDoc["outputDampen"];
        outputCutoff = jsonDoc["outputCutoff"];
      } else {
        Serial.println(String("loadOutputChannels() sysVersion mismatch. Got: ") + sysVersion + String(", expecting: ") + String(SYSTEM_VERSION));
        Serial.println("Not loading saved parameters, falling back to defaults.");
      }
    } else {
      Serial.print("loadOutputChannels() deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    f.close();
  }

  if (timerActive) {
    timerStart(adcTimer);
  }
}
