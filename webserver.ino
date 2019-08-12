#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "DNSServer.h"
#include <ArduinoJson.h>

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
    if (req->hasHeader("Host")) {
      AsyncWebHeader *h = req->getHeader("Host");
      if (h->value() != WiFi.softAPIP().toString()) {
        Serial.println(String("Host: ") + h->value());
        AsyncWebServerResponse *resp = req->beginResponse(302);
        String loc = String("http://") + WiFi.softAPIP().toString() + String("/");
        resp->addHeader("Location", loc);
        req->send(resp);
        Serial.println(String("Redirected to ") + loc);
        return;
      }
    }
    req->send_P(200, "text/html", index_html, indexTplProcessor);
  });

  web.on("/bulma.min.css", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request for bulma.min.css");
    AsyncWebServerResponse *resp = req->beginResponse_P(200, "text/css", bulma_min_css_gz, bulma_min_css_gz_len);
    resp->addHeader("Content-Encoding", "gzip");
    req->send(resp);
  });
  
  web.on("/bulma-slider.min.js", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request for bulma-slider.min.js");
    AsyncWebServerResponse *resp = req->beginResponse_P(200, "application/javascript", bulma_slider_min_js_gz, bulma_slider_min_js_gz_len);
    resp->addHeader("Content-Encoding", "gzip");
    req->send(resp);
  });

  web.on("/bulma-slider.min.css", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request for bulma-slider.min.css");
    AsyncWebServerResponse *resp = req->beginResponse_P(200, "text/css", bulma_slider_min_css_gz, bulma_slider_min_css_gz_len);
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
    StaticJsonDocument<2048> jsonDoc;
    JsonObject root = jsonDoc.to<JsonObject>();
    
    root["millis"] = millis();
    root["sysVersion"] = SYSTEM_VERSION;
    root["inputBoost"] = inputBoost;
    root["outputDampen"] = outputDampen;
    root["outputCutoff"] = outputCutoff;
    serializeOutputChannels(root);
    
    AsyncResponseStream *resp = req->beginResponseStream("application/json");
    serializeJson(jsonDoc, *resp);
    req->send(resp);
  });

  // Saves outputs configuration
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
    //Serial.print("RAW JSON: ");
    //Serial.println((char*)data);
    
    StaticJsonDocument<2048> jsonDoc;
    auto error = deserializeJson(jsonDoc, data);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    deserializeOutputChannels(jsonDoc, outputChannelConfigs);

    inputBoost = jsonDoc["inputBoost"];
    outputDampen = jsonDoc["outputDampen"];
    outputCutoff = jsonDoc["outputCutoff"];
    
    saveChannelConfig();
    configModeMessage = "** Saved! **";
    configModeMessageTime = millis();
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

void disableWiFi() {
  Serial.println("Turning WiFi off");
  WiFi.mode(WIFI_OFF);
}
