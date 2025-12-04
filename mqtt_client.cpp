// mqtt_client.cpp
#include "mqtt_client.h"
#include <WiFi.h>
#include <PubSubClient.h>

static WiFiClient espClient;
static PubSubClient mqtt(espClient);

// Stored config
static const char* s_topic = nullptr;

static void mqttReconnect() {
  while (!mqtt.connected()) {
    String clientId = "ESP32Water-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (!mqtt.connect(clientId.c_str())) {
      delay(2000);
    }
  }
}

void mqttSetup(const char* broker, uint16_t port, const char* topic) {
  s_topic = topic;
  mqtt.setServer(broker, port);
}

void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!mqtt.connected()) {
    mqttReconnect();
  }
  mqtt.loop();
}

void mqttPublishFlow(float flow_lpm, double total_litres) {
  if (!mqtt.connected()) return;

  String payload = "{";
  payload += "\"flow_lpm\":" + String(flow_lpm, 3) + ",";
  payload += "\"total_litres\":" + String(total_litres, 6);
  payload += "}";

  mqtt.publish(s_topic, payload.c_str());
}
