/*
  flow_meter.ino
  ESP32 (Arduino) sketch to measure water flow from a YF-S403-like Hall sensor.

  - Uses interrupt on CHANGE to count both rising + falling edges.
  - Converts edges -> Hz -> L/min using datasheet constant (Hz = 7.5 * Q).
  - Integrates to get running volume in litres.
  - Publishes flow + total litres via MQTT to a broker (e.g., Raspberry Pi).

  Wiring (typical):
    Sensor VCC  -> 5V (or 3.3V if rated)
    Sensor GND  -> GND
    Sensor OUT  -> ESP32 GPIO (FLOW_PIN), with pull-up as needed

  NOTE:
    Adjust WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_IP, FLOW_PIN as per your setup.
*/

#include <Arduino.h>
#include <WiFi.h>
#include "mqtt_client.h"

// ----------------- User configuration -----------------

// Flow sensor pin
const uint8_t FLOW_PIN = 18;                 // GPIO connected to sensor OUT
const uint32_t MEASURE_INTERVAL_MS = 100;    // compute flow every 100 ms

// The YF-S403 datasheet states: Hz = 7.5 * Q(L/min)
const float HZ_PER_L_PER_MIN = 7.5f;

// If true, we count both rising + falling edges on FLOW_PIN.
// That means "edges" = 2 * "pulses". Many hall sensors produce one full cycle per pulse.
const bool COUNT_EDGES = true;

// WiFi config
const char* WIFI_SSID     = "RTUniverse";
const char* WIFI_PASSWORD = "8754820702";

// MQTT broker (e.g., Raspberry Pi) config
const char* MQTT_BROKER_IP = "192.168.1.140";    // CHANGE to your Pi's IP
const uint16_t MQTT_PORT   = 1883;
const char* MQTT_TOPIC     = "home/water/main";

// ----------------- Globals -----------------

// Volatile edge counter updated in ISR
volatile uint32_t g_edgeCount = 0;

// Time of last measurement (ms)
uint32_t lastMeasureMillis = 0;

// Running total volume in litres
double totalLiters = 0.0;

// ----------------- Interrupt handler -----------------

// ISR: called on every edge of FLOW_PIN
void IRAM_ATTR flowISR() {
  g_edgeCount++;
}

// Safely read and clear edge counter
uint32_t readAndClearEdgeCount() {
  uint32_t edges;
  noInterrupts();
  edges = g_edgeCount;
  g_edgeCount = 0;
  interrupts();
  return edges;
}

// ----------------- Setup -----------------

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("=== ESP32 Flow Meter + MQTT ===");

  // ---- WiFi ----
  Serial.print("Connecting to WiFi: ");
  Serial.print(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 60) { // ~30s timeout
    delay(500);
    Serial.print(".");
    tries++;
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect FAILED, continuing anyway (MQTT will keep retrying).");
  }

  // ---- MQTT ----
  mqttSetup(MQTT_BROKER_IP, MQTT_PORT, MQTT_TOPIC);

  // ---- Flow sensor pin + interrupt ----
  pinMode(FLOW_PIN, INPUT_PULLUP); // typical for open-collector output

  if (COUNT_EDGES) {
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, CHANGE);
  } else {
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);
  }

  lastMeasureMillis = millis();

  Serial.println("Flow meter initialized.");
  Serial.print("FLOW_PIN=");
  Serial.println(FLOW_PIN);
  Serial.print("MEASURE_INTERVAL_MS=");
  Serial.println(MEASURE_INTERVAL_MS);
}

// ----------------- Main loop -----------------

void loop() {
  // Keep MQTT connection alive
  mqttLoop();

  uint32_t now = millis();
  if (now - lastMeasureMillis >= MEASURE_INTERVAL_MS) {

    // Snapshot edges for this interval
    uint32_t edges = readAndClearEdgeCount();

    // Interval in seconds
    float interval_s = (now - lastMeasureMillis) / 1000.0f;
    if (interval_s <= 0.0f) {
      interval_s = MEASURE_INTERVAL_MS / 1000.0f;
    }

    // Convert edges to "cycles" (Hz); if counting both edges, cycles = edges / 2
    float edges_per_sec = edges / interval_s;
    float cycles_per_sec = COUNT_EDGES ? (edges_per_sec / 2.0f) : edges_per_sec;

    // According to datasheet: Hz = 7.5 * Q(L/min) -> Q = Hz / 7.5
    float flow_L_per_min = cycles_per_sec / HZ_PER_L_PER_MIN;
    float flow_L_per_sec = flow_L_per_min / 60.0f;

    // Volume this interval (L)
    double volume_this_interval_L = (double)flow_L_per_sec * (double)interval_s;
    totalLiters += volume_this_interval_L;

    // Debug print
    Serial.print("edges=");
    Serial.print(edges);
    Serial.print("  interval_s=");
    Serial.print(interval_s, 3);
    Serial.print("  cycles_per_sec=");
    Serial.print(cycles_per_sec, 3);
    Serial.print("  flow(L/min)=");
    Serial.print(flow_L_per_min, 3);
    Serial.print("  interval_L=");
    Serial.print(volume_this_interval_L, 6);
    Serial.print("  total_L=");
    Serial.println(totalLiters, 6);

    // ---- Publish to MQTT ----
    mqttPublishFlow(flow_L_per_min, totalLiters);

    lastMeasureMillis = now;
  }

  // Short delay to avoid busy-looping; interrupts still fire
  delay(10);
}
