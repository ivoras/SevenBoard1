#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "DNSServer.h"

const char* wifiSSID = "SEVENBOARD1";
const char* wifiPassword = "grillaj0";

AsyncWebServer web(80);
IPAddress apIP(192, 168, 1, 1);
DNSServer apDNSServer;

void setupWiFi() {
  Serial.println("Setting up SPIFFS");
  if (!SPIFFS.begin()){
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
  }
  
  Serial.println("Setting up WiFi");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(wifiSSID, wifiPassword);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  apDNSServer.start(53, "*", apIP);

  web.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    Serial.println("Got request");
    req->send(SPIFFS, "/index.html", "text/html");
  });

  web.begin();
  Serial.println("Web server started");
}

void webserverTasks() {
  apDNSServer.processNextRequest();

  
}
