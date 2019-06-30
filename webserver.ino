#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "DNSServer.h"
#include <ArduinoJson.h>

const char* wifiSSID = "SEVENBOARD1";
const char* wifiPassword = "grillaj0";

AsyncWebServer web(80);
IPAddress apIP(10, 10, 10, 1);
DNSServer apDNSServer;
bool webServerSetup = false;

String indexTplProcessor(const String& var) {
  if (var == "SYSTEM_VERSION")
    return SYSTEM_VERSION;
  return String();
}

void setupWiFi() {
  // Unfortunately, SPIFFS is broken with hardware timers
  
  Serial.println("Setting up WiFi");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(wifiSSID, wifiPassword);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  apDNSServer.start(53, "*", apIP);

  web.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });

  web.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request for index.html");
    req->send_P(200, "text/html", index_html, indexTplProcessor);
  });

  web.on("/bulma.min.css", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request for bulma.min.css");
    AsyncWebServerResponse *resp = req->beginResponse_P(200, "text/css", bulma_min_css_gz, bulma_min_css_gz_len);
    resp->addHeader("Content-Encoding", "gzip");
    req->send(resp);
  });

  web.on("/knockout.js", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request for knockout.js");
    AsyncWebServerResponse *resp = req->beginResponse_P(200, "application/javascript", knockout_3_5_0_js_gz, knockout_3_5_0_js_gz_len);
    resp->addHeader("Content-Encoding", "gzip");
    req->send(resp);
  });

  web.on("/outputs", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("GET /outputs");
    StaticJsonDocument<1024> jsonDoc;
    JsonObject root = jsonDoc.to<JsonObject>();
    
    root["millis"] = millis();
    root["sysVersion"] = SYSTEM_VERSION;

    JsonArray outputs = root.createNestedArray("outputs");
    for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
      JsonObject output = outputs.createNestedObject();
      output["i"] = i;
      output["fslot_r"] = outputChannelFreqs[i].fslot_r;
      output["fslot_g"] = outputChannelFreqs[i].fslot_g;
      output["fslot_b"] = outputChannelFreqs[i].fslot_b;
      output["fwidth_r"] = outputChannelFreqs[i].fwidth_r;
      output["fwidth_g"] = outputChannelFreqs[i].fwidth_g;
      output["fwidth_b"] = outputChannelFreqs[i].fwidth_b;
      output["show_lines"] = outputChannelFreqs[i].showLines;
    }
    
    AsyncResponseStream *resp = req->beginResponseStream("application/json");
    serializeJson(jsonDoc, *resp);
    req->send(resp);
  });

  web.on("/outputs", HTTP_POST, [](AsyncWebServerRequest *req) {
    Serial.println("POST /outputs (req)");
  }, 
  NULL, 
  [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    Serial.println("POST /outputs (body)");
    if (index + len != total) {
      Serial.println("/outputs body handler got partial data?");
      return;
    }
    Serial.print("RAW JSON: ");
    Serial.println((char*)data);
    
    StaticJsonDocument<1024> jsonDoc;
    auto error = deserializeJson(jsonDoc, data);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    
    JsonArray outputs = jsonDoc["outputs"];
    for (int n = 0; n < outputs.size(); n++) {
      int i = outputs[n]["i"];
      outputChannelFreqs[i].fslot_r = outputs[n]["fslot_r"];
      outputChannelFreqs[i].fslot_g = outputs[n]["fslot_g"];
      outputChannelFreqs[i].fslot_b = outputs[n]["fslot_b"];

      outputChannelFreqs[i].fwidth_r = outputs[n]["fwidth_r"];
      outputChannelFreqs[i].fwidth_g = outputs[n]["fwidth_g"];
      outputChannelFreqs[i].fwidth_b = outputs[n]["fwidth_b"];

      outputChannelFreqs[i].showLines = outputs[n]["show_lines"];
    }
    dumpOutputChannels();
  });

  webServerSetup = true;
  web.begin();
  Serial.println("Web server started");
}

void webserverTasks() {
  if (!webServerSetup)
    return;
  apDNSServer.processNextRequest();

  
}
