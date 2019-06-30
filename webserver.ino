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
    Serial.println("Got request for /getfreqs");
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
/*
  web.on("/outputs", HTTP_POST, [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
    
  });
*/
  webServerSetup = true;
  web.begin();
  Serial.println("Web server started");
}

void webserverTasks() {
  if (!webServerSetup)
    return;
  apDNSServer.processNextRequest();

  
}
