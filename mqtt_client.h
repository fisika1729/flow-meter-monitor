#pragma once
#include <Arduino.h>

void mqttSetup(const char* broker, uint16_t port, const char* topic);
void mqttLoop();
void mqttPublishFlow(float flow_lpm, double total_litres);
