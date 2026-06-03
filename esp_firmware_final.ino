// ============================================================
//  ONION DIGITAL TWIN — Final Production Firmware
//  Board   : ESP32 Dev Module
//
//  Sensors:
//    DHT22  → GPIO15  (Temperature + Humidity)
//    MQ-135 → GPIO27  (Gas / Rot detection, analog)
//
//  Actuators — 2 RELAYS ONLY (Active-LOW: LOW=ON, HIGH=OFF):
//    GPIO26 → RELAY 1 → FAN
//    GPIO25 → RELAY 2 → NEEM VAPOR SPRAY
//
//  NO HUMIDIFIER — relay 2 is neem only.
//
//  FAN LOGIC:
//    ON  when humidity > 75%       (too much moisture)
//    ON  when gas > 1200 raw       (rot gases detected)
//    ON  during neem burst         (circulate vapor in box)
//    ON  for 2min after neem ends  (purge spray humidity)
//    30s hysteresis before turning OFF (no rapid flapping)
//
//  NEEM VAPOR LOGIC:
//    Triggers when gas > 2250 raw
//    Sprays for 30 seconds
//    Then LOCKED OUT for 10 minutes (neem dissipates, humidity settles)
//    Will NOT re-trigger during lockout even if gas stays high
//    Fan runs during + after burst to spread and then purge vapor
//
//  WiFi: POST JSON to http://10.201.29.214:3001/data every 2s
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <ArduinoJson.h>

// ── WiFi ─────────────────────────────────────────────────────
const char* ssid      = "OPPO_A53";
const char* password  = "dhruthi128";
const char* serverURL = "http://10.201.29.214:3001/data";

// ── Pins ─────────────────────────────────────────────────────
#define DHT_PIN     18    // DHT22 data pin
#define DHT_TYPE    DHT22
#define MQ135_PIN   34    // MQ-135 analog input
#define FAN_RELAY   26    // Relay 1 → Fan
#define NEEM_RELAY  25    // Relay 2 → Neem vapor spray

// ── Thresholds ───────────────────────────────────────────────
// Changed to raw gas sensor values based on 0-4095 range
const float HUM_HIGH           = 75.0;     // humidity % → fan on
const float GAS_FAN_THRESHOLD  = 1200.0;   // Raw gas value → fan on
const float GAS_NEEM_THRESHOLD = 30.0;   // Raw gas value → neem burst

// ── Timing ───────────────────────────────────────────────────
const unsigned long NEEM_BURST_MS    = 300000UL;  // 30s  neem ON
const unsigned long NEEM_LOCKOUT_MS  = 600000UL; // 10min no re-spray
const unsigned long FAN_POST_NEEM_MS = 120000UL; // 2min fan after neem
const unsigned long FAN_HYSTERESIS_MS= 30000UL;  // 30s fan stays ON after clear

// ── Neem state ───────────────────────────────────────────────
bool          neemBursting     = false;
bool          neemLockout      = false;
unsigned long neemStartMs      = 0;
unsigned long neemBurstEndMs   = 0;
unsigned long neemLockoutEndMs = 0;
int           neemBurstCount   = 0;

// ── Fan state ────────────────────────────────────────────────
bool          fanActive        = false;
bool          fanInHysteresis  = false;
unsigned long fanClearStartMs  = 0;

// ── Gas smoothing (5-sample rolling average) ─────────────────
float gasBuffer[5]  = {0,0,0,0,0};
int   gasBufferIdx  = 0;

DHT dht(DHT_PIN, DHT_TYPE);

// ── Relay helpers (active LOW) ────────────────────────────────
void relayON (int pin) { digitalWrite(pin, LOW);  }
void relayOFF(int pin) { digitalWrite(pin, HIGH); }

// ── Smoothed gas reading ──────────────────────────────────────
float readGasSmoothed() {
  // Read raw analog value (0-4095)
  float raw = (float)analogRead(MQ135_PIN);
  gasBuffer[gasBufferIdx] = raw;
  gasBufferIdx = (gasBufferIdx + 1) % 5;
  float sum = 0;
  for (int i = 0; i < 5; i++) sum += gasBuffer[i];
  return sum / 5.0f;
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // Both relays OFF at boot
  pinMode(FAN_RELAY,  OUTPUT); relayOFF(FAN_RELAY);
  pinMode(NEEM_RELAY, OUTPUT); relayOFF(NEEM_RELAY);

  dht.begin();

  // Warm up gas smoothing buffer with raw readings
  for (int i = 0; i < 5; i++) {
    gasBuffer[i] = (float)analogRead(MQ135_PIN);
    delay(50);
  }

  Serial.println("\n[BOOT] Onion Digital Twin");
  Serial.println("[PINS] DHT22=GPIO15  MQ135=GPIO27  Fan=GPIO26  Neem=GPIO25");
  Serial.println("[INFO] 2 relays: Fan + Neem only. No humidifier.");
  Serial.println("[INFO] Neem: 30s burst → 10min lockout");
  Serial.print  ("[BOOT] Connecting WiFi");

  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print("."); tries++;
  }

  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\n[WiFi] Connected → " + WiFi.localIP().toString());
  else
    Serial.println("\n[WiFi] Offline — actuators still work locally");
}

// ── Main loop ────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── STEP 1: Read sensors ─────────────────────────────────
  float humidity    = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("[ERROR] DHT22 read failed — retrying");
    delay(2000);
    return; // relays hold last state — safe to skip one cycle
  }

  float gasLevel = readGasSmoothed();

  // ── STEP 2: Neem state machine ────────────────────────────
  //
  //  IDLE ──── gas > threshold ────► BURSTING (30s)
  //  BURSTING ─ 30s elapsed ───► LOCKOUT (10min)
  //  LOCKOUT ── 10min elapsed ─► IDLE
  //
  if (neemBursting) {

    if (now - neemStartMs >= NEEM_BURST_MS) {
      // Burst complete — stop spray, enter lockout
      neemBursting     = false;
      neemLockout      = true;
      neemBurstEndMs   = now;
      neemLockoutEndMs = now + NEEM_LOCKOUT_MS;
      relayOFF(NEEM_RELAY);
      Serial.println("[NEEM] Burst done → 10min lockout started");
    }
    // Still within burst window — relay stays ON (applied in Step 4)

  } else if (neemLockout) {

    if (now >= neemLockoutEndMs) {
      neemLockout = false;
      Serial.println("[NEEM] Lockout done — ready");
    }

  } else {
    // IDLE — check if new burst needed
    if (gasLevel > GAS_NEEM_THRESHOLD) {
      neemBursting = true;
      neemStartMs  = now;
      neemBurstCount++;
      relayON(NEEM_RELAY);
      Serial.printf("[NEEM] Burst #%d started — gas=%.1f raw\n",
                    neemBurstCount, gasLevel);
    }
  }

  // ── STEP 3: Fan logic ─────────────────────────────────────
  //
  //  Turns ON for any of these reasons:
  //    A) Humidity too high → vent moisture out of box
  //    B) Gas too high      → flush rot gases out of box
  //    C) Neem bursting     → circulate vapor around onions
  //    D) Post-neem purge   → remove excess humidity from spray
  //
  //  Hysteresis: once all conditions clear, stays ON 30s more
  //  before switching OFF — prevents rapid relay flapping
  //
  bool postNeemPurge = (!neemBursting &&
                        neemBurstEndMs > 0 &&
                        (now - neemBurstEndMs) < FAN_POST_NEEM_MS);

  bool fanNeedsOn = (humidity > HUM_HIGH)          ||  // A
                    (gasLevel > GAS_FAN_THRESHOLD)  ||  // B
                    neemBursting                    ||  // C
                    postNeemPurge;                      // D

  if (fanNeedsOn) {
    // Condition active — fan ON, reset hysteresis
    fanActive       = true;
    fanInHysteresis = false;
    fanClearStartMs = 0;

  } else if (fanActive) {
    // All conditions cleared — start hysteresis countdown
    if (!fanInHysteresis) {
      fanInHysteresis = true;
      fanClearStartMs = now;
    }
    if ((now - fanClearStartMs) >= FAN_HYSTERESIS_MS) {
      fanActive       = false;
      fanInHysteresis = false;
      Serial.println("[FAN] All clear — OFF");
    }
  }

  // ── STEP 4: Apply relays ──────────────────────────────────
  fanActive    ? relayON(FAN_RELAY)  : relayOFF(FAN_RELAY);
  neemBursting ? relayON(NEEM_RELAY) : relayOFF(NEEM_RELAY);

  // ── STEP 5: Health score ──────────────────────────────────
  // Adjusted health score calculation to handle raw gas value range roughly equivalent to old 0-100 logic.
  // We approximate the old 30ppm threshold by dividing raw by 40.95.
  float gasScoreLoss = max(0.0f, ((gasLevel / 40.95f) - 30.0f) * 0.8f);
  float health = 100.0f
    - max(0.0f, (humidity    - 65.0f) * 1.5f)
    - max(0.0f, (temperature - 25.0f) * 2.0f)
    - gasScoreLoss;
  health = constrain(health, 0.0f, 100.0f);

  // ── STEP 6: Serial monitor ────────────────────────────────
  Serial.println("═══════════════════════════════════════════");
  Serial.printf("[SENSOR] Temp:%.1f°C  Hum:%.1f%%  Gas:%.1f raw\n",
                temperature, humidity, gasLevel);
  Serial.printf("[HEALTH] %.1f / 100\n", health);

  Serial.print("[FAN]    ");
  if (fanActive) {
    Serial.print("ON");
    if (humidity > HUM_HIGH)          Serial.print(" ← high humidity");
    if (gasLevel > GAS_FAN_THRESHOLD) Serial.print(" ← gas detected");
    if (neemBursting)                 Serial.print(" ← spreading neem");
    if (postNeemPurge)                Serial.print(" ← post-neem purge");
    if (fanInHysteresis)              Serial.printf(" ← hysteresis %lus left",
                                        (FAN_HYSTERESIS_MS - (now - fanClearStartMs)) / 1000);
  } else {
    Serial.print("off");
  }
  Serial.println();

  Serial.print("[NEEM]   ");
  if (neemBursting)
    Serial.printf("BURSTING — %lus left\n",
                  (NEEM_BURST_MS - (now - neemStartMs)) / 1000);
  else if (neemLockout)
    Serial.printf("LOCKOUT — %lus left  (total bursts: %d)\n",
                  (neemLockoutEndMs - now) / 1000, neemBurstCount);
  else
    Serial.printf("idle  (total bursts: %d)\n", neemBurstCount);

  // ── STEP 7: POST to dashboard ─────────────────────────────
  StaticJsonDocument<256> doc;
  doc["temperature"]      = round(temperature * 10.0f) / 10.0f;
  doc["humidity"]         = round(humidity    * 10.0f) / 10.0f;
  doc["gasLevel"]         = (int)round(gasLevel);
  doc["healthScore"]      = round(health * 10.0f) / 10.0f;
  doc["fanActive"]        = fanActive;
  doc["humidifierActive"] = false;        // no humidifier in this build
  doc["neemActive"]       = neemBursting;
  doc["neemBursts"]       = neemBurstCount;
  doc["neemLockout"]      = neemLockout;

  String payload;
  serializeJson(doc, payload);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(payload);
    Serial.printf("[HTTP]   %s (code %d)\n", code > 0 ? "OK" : "FAIL", code);
    http.end();
  } else {
    Serial.println("[WiFi]   Offline — reconnecting...");
    WiFi.reconnect();
  }

  delay(2000);
}