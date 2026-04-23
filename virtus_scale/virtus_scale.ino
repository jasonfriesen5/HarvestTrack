// ══════════════════════════════════════════════════════════════
//  Virtus Harvest BLE Scale Firmware v2.0.0
//  Hardware: ESP32 + HX711 Load Cell Amplifier
//  BLE Protocol: Nordic UART Service — matches Virtus Harvest app
//  WiFi: Access point for remote view-only devices
//
//  FIRST FLASH: Must be via USB cable
//  Future updates: BLE OTA from Virtus app
// ══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <Update.h>
#include "HX711.h"

// ── FIRMWARE VERSION ──────────────────────────────────────────
// Bump this each time you push a new update
#define FIRMWARE_VERSION  "2.1.0"
#define DEVICE_MODEL      "VM#1"

// ── WiFi ACCESS POINT ─────────────────────────────────────────
// ESP32 creates its own WiFi — connect any device and open 192.168.4.1
#define WIFI_AP_SSID  "VM#1"    // Network name shown on devices
// No password — open network for easy cab access

// ── PIN CONFIGURATION ──────────────────────────────────────────
#define DT_PIN   27    // HX711 Data  → GPIO27
#define SCK_PIN  26    // HX711 Clock → GPIO26

// Optional voltage sensing (wire voltage dividers to these pins)
#define PIN_BATT_ADC        34
#define PIN_EXT_ADC         35
#define BATT_DIVIDER_RATIO  2.0f   // 100k/100k divider for LiPo
#define EXT_DIVIDER_RATIO  11.0f   // 100k/10k divider for 12V
#define ADC_REF_V           3.3f
#define ADC_MAX          4095.0f

// ── BLE UUIDs (Nordic UART Service) ───────────────────────────
#define SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ── EEPROM LAYOUT ─────────────────────────────────────────────
#define EEPROM_SIZE       16
#define ADDR_CALIBRATION   0   // float (4 bytes) — calibration factor
#define ADDR_OFFSET        4   // float (4 bytes) — tare offset
#define ADDR_LASTUNLOAD    8   // int   (4 bytes) — saved unload total
#define ADDR_THRESHOLD    12   // int   (4 bytes) — stability tolerance

// ── SCALE CONFIG ───────────────────────────────────────────────
#define MEDIAN_SAMPLES   3     // Readings to median-filter
#define MAX_JUMP       200.0f  // Max kg change before spike detection

// ── TIMING ────────────────────────────────────────────────────
const unsigned long SEND_INTERVAL    =  500;   // BLE weight send rate (ms)
const unsigned long COMPARE_INTERVAL = 1500;   // Stability check rate (ms)
const unsigned long STATUS_INTERVAL  = 10000;  // Battery status rate (ms)

// ── GLOBALS ───────────────────────────────────────────────────
WebServer wifiServer(80);

HX711              scale;
BLEServer*         pServer         = nullptr;
BLECharacteristic* pTxChar         = nullptr;
BLECharacteristic* pRxChar         = nullptr;
bool               deviceConnected = false;
bool               oldConnected    = false;

float calibrationFactor = 61.69f;
float offset            = 0.0f;
int   threshold         = 11;
int   lastUnload        = 0;
int   totalUnloaded     = 0;

float currentWeight     = 0.0f;
float lastValid         = 0.0f;
int   compareWeight     = 0;
int   unloadStartWeight = 0;
int   spikeCount        = 0;
bool  hx711WasReset     = false;
bool  unloading         = false;
bool  stable            = false;

unsigned long lastSendTime    = 0;
unsigned long lastCompareTime = 0;
unsigned long lastStatusTime  = 0;

const int MAX_SPIKES = 3;

// OTA
bool   otaInProgress   = false;
size_t otaSize         = 0;
size_t otaReceived     = 0;
volatile bool pendingRestart  = false;   // set from BLE callback, acted on in loop()
unsigned long pendingRestartAt = 0;       // millis() time at which to restart

// ── FORWARD DECLARATIONS ──────────────────────────────────────
void  sendLine(const String& msg);
void  sendInfo();
void  sendStatus();
float readHX711Weight();
void  resetHX711();
long  adaptiveRound(float value);
float readVoltage(int pin, float ratio);
int   voltageToBattPct(float v);

// ── BLE SERVER CALLBACKS ──────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    Serial.println("App connected via BLE");
    delay(300); sendInfo();
    delay(200); sendStatus();
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    otaInProgress   = false;
    Serial.println("App disconnected — restarting advertising");
  }
};

// ── BLE RX CALLBACKS ──────────────────────────────────────────
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {

    // When OTA is active, incoming bytes are raw firmware binary
    if (otaInProgress) {
      String val = pChar->getValue();
      if (val.length() > 0) {
        size_t written = Update.write((uint8_t*)val.c_str(), val.length());
        otaReceived += written;
        int pct = (int)((otaReceived * 100UL) / otaSize);
        static int lastPct = -1;
        if (pct / 5 != lastPct / 5) {
          lastPct = pct;
          sendLine("OTA_PROGRESS:" + String(pct));
        }
        if (otaReceived >= otaSize) {
          otaInProgress = false;
          // Notify app that we received everything and are now verifying
          sendLine("OTA_VERIFY");
          delay(150); // give BLE stack time to transmit
          if (Update.end(true)) {
            sendLine("OTA_OK");
            Serial.println("OTA complete — scheduling restart");
            // IMPORTANT: Do NOT call ESP.restart() here inside the BLE callback.
            // The BLE stack needs time to flush the OTA_OK notification out before
            // the radio goes down. We flag a restart and let loop() handle it so the
            // callback can return, notification can be sent, and the stack can idle.
            pendingRestartAt = millis() + 1200;
            pendingRestart   = true;
          } else {
            sendLine("OTA_FAIL:VERIFY");
            Serial.println("OTA verify failed");
          }
        }
      }
      return;
    }

    // Text commands from app
    String cmd = String(pChar->getValue().c_str());
    cmd.trim();
    if (!cmd.length()) return;
    Serial.print("CMD: "); Serial.println(cmd);

    if (cmd == "TARE") {
      offset = readHX711Weight();
      EEPROM.put(ADDR_OFFSET, offset);
      EEPROM.commit();
      sendLine("0.00");
      Serial.println("Scale tared");

    } else if (cmd == "UNLOAD") {
      unloading         = true;
      unloadStartWeight = (int)currentWeight;
      Serial.println("Unload started");

    } else if (cmd == "STOP") {
      unloading  = false;
      lastUnload = totalUnloaded;
      EEPROM.put(ADDR_LASTUNLOAD, lastUnload);
      EEPROM.commit();
      Serial.println("Unload stopped");

    } else if (cmd == "RESET_TOTAL") {
      totalUnloaded = 0;
      lastUnload    = 0;
      EEPROM.put(ADDR_LASTUNLOAD, lastUnload);
      EEPROM.commit();
      Serial.println("Total reset");

    } else if (cmd == "INFO") {
      sendInfo();

    } else if (cmd == "STATUS") {
      sendStatus();

    } else if (cmd == "OTA_ABORT") {
      Update.abort();
      otaInProgress = false;
      sendLine("OTA_ABORTED");

    } else if (cmd.startsWith("OTA_BEGIN:")) {
      otaSize     = (size_t)cmd.substring(10).toInt();
      otaReceived = 0;
      if (otaSize > 0 && Update.begin(otaSize)) {
        otaInProgress = true;
        sendLine("OTA_READY");
        Serial.printf("OTA: expecting %u bytes\n", (unsigned)otaSize);
      } else {
        sendLine("OTA_FAIL:BEGIN");
        Serial.println("OTA begin failed");
      }

    } else if (cmd.startsWith("CAL:")) {
      calibrationFactor = cmd.substring(4).toFloat();
      scale.set_scale(calibrationFactor);
      EEPROM.put(ADDR_CALIBRATION, calibrationFactor);
      EEPROM.commit();
      sendLine("CAL_OK:" + String(calibrationFactor, 4));
      Serial.print("Cal factor: "); Serial.println(calibrationFactor);

    } else if (cmd.startsWith("OFFSET:")) {
      offset = cmd.substring(7).toFloat();
      EEPROM.put(ADDR_OFFSET, offset);
      EEPROM.commit();
      Serial.print("Offset set: "); Serial.println(offset);

    } else if (cmd.startsWith("STABLETOL:")) {
      threshold = cmd.substring(10).toInt();
      EEPROM.put(ADDR_THRESHOLD, threshold);
      EEPROM.commit();
      Serial.print("Stable tolerance: "); Serial.println(threshold);
    }
  }
};

// ── SEND LINE TO BLE APP ──────────────────────────────────────
void sendLine(const String& msg) {
  if (!deviceConnected || !pTxChar) return;
  String out = msg + "\n";
  pTxChar->setValue((uint8_t*)out.c_str(), out.length());
  pTxChar->notify();
}

// ── INFO PACKET ───────────────────────────────────────────────
void sendInfo() {
  uint64_t chipId = ESP.getEfuseMac();
  char serial[13];
  snprintf(serial, sizeof(serial), "%04X%08X",
           (uint16_t)(chipId >> 32), (uint32_t)chipId);
  String msg = "INFO:firmware=" + String(FIRMWARE_VERSION)
             + ",model="   + String(DEVICE_MODEL)
             + ",serial="  + String(serial)
             + ",update=none";
  sendLine(msg);
}

// ── STATUS PACKET ─────────────────────────────────────────────
void sendStatus() {
  float battV = readVoltage(PIN_BATT_ADC, BATT_DIVIDER_RATIO);
  float extV  = readVoltage(PIN_EXT_ADC,  EXT_DIVIDER_RATIO);
  int   pct   = voltageToBattPct(battV);
  int   chrg  = (extV > 4.5f && pct < 95) ? 1 : 0;
  char  buf[80];
  snprintf(buf, sizeof(buf),
    "STATUS:batt=%d,batt_v=%.2f,ext_v=%.1f,charging=%d",
    pct, battV, extV, chrg);
  sendLine(String(buf));
}

// ── VOLTAGE READING ───────────────────────────────────────────
float readVoltage(int pin, float ratio) {
  long sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(pin); delay(2); }
  return (sum / 16.0f / ADC_MAX) * ADC_REF_V * ratio;
}

// ── LIPO BATTERY % (linearized between voltage knee-points) ──
int voltageToBattPct(float v) {
  // LiPo discharge curve with linear interpolation between points
  // Voltage → Percent pairs based on typical LiPo discharge profile
  static const float vTable[] = { 4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.75f, 3.70f, 3.65f, 3.60f, 3.50f, 3.40f, 3.30f };
  static const int   pTable[] = {   100,    90,    80,    70,    55,    45,    35,    25,    18,    10,     5,     0 };
  static const int   nPts     = sizeof(vTable) / sizeof(vTable[0]);

  if (v >= vTable[0]) return 100;
  if (v <= vTable[nPts - 1]) return 0;
  for (int i = 0; i < nPts - 1; i++) {
    if (v >= vTable[i + 1]) {
      float frac = (v - vTable[i + 1]) / (vTable[i] - vTable[i + 1]);
      return pTable[i + 1] + (int)(frac * (pTable[i] - pTable[i + 1]) + 0.5f);
    }
  }
  return 0;
}

// ── HX711 READ WITH MEDIAN FILTER + SPIKE REJECTION ──────────
float readHX711Weight() {
  if (!scale.is_ready()) {
    unsigned long t = millis();
    while (!scale.is_ready() && millis() - t < 300) {}
    if (!scale.is_ready()) {
      resetHX711();
      delay(100);
      if (!scale.is_ready()) return lastValid;
      hx711WasReset = true;
    }
  }

  float r[MEDIAN_SAMPLES];
  for (int i = 0; i < MEDIAN_SAMPLES; i++) {
    unsigned long tw = millis();
    while (!scale.is_ready() && millis() - tw < 200) {}
    if (!scale.is_ready()) return lastValid;  // bail if HX711 hangs mid-sample
    r[i] = scale.get_units(1);
    delay(2);
  }

  // Insertion sort for median
  for (int i = 1; i < MEDIAN_SAMPLES; i++) {
    float key = r[i];
    int j = i - 1;
    while (j >= 0 && r[j] > key) { r[j + 1] = r[j]; j--; }
    r[j + 1] = key;
  }
  float med = r[MEDIAN_SAMPLES / 2];

  if (abs(med - lastValid) > MAX_JUMP && !hx711WasReset) {
    spikeCount++;
    if (spikeCount < MAX_SPIKES) {
      Serial.printf("Spike ignored (%d)\n", spikeCount);
      return lastValid;
    }
    spikeCount = 0;
    Serial.println("Too many spikes — accepting new value");
  } else {
    spikeCount = 0;
  }

  hx711WasReset = false;
  lastValid     = med;
  return med;
}

// ── RESET HX711 VIA CLOCK PULSES ─────────────────────────────
void resetHX711() {
  hx711WasReset = true;
  pinMode(SCK_PIN, OUTPUT);
  for (int i = 0; i < 65; i++) {
    digitalWrite(SCK_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(SCK_PIN, LOW);  delayMicroseconds(1);
  }
}

// ── ADAPTIVE ROUNDING ─────────────────────────────────────────
long adaptiveRound(float value) {
  float absVal = fabsf(value);
  long  sign   = (value < 0.0f) ? -1L : 1L;
  long  rounded;
  if (absVal <= 500.0f) rounded = (long)(roundf(absVal / 10.0f) * 10.0f);
  else                  rounded = (long)(roundf(absVal / 20.0f) * 20.0f);
  return sign * rounded;
}

// ── SETUP ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Virtus Scale v" FIRMWARE_VERSION " ===");

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(ADDR_CALIBRATION, calibrationFactor);
  EEPROM.get(ADDR_OFFSET,      offset);
  EEPROM.get(ADDR_LASTUNLOAD,  lastUnload);
  EEPROM.get(ADDR_THRESHOLD,   threshold);
  totalUnloaded = lastUnload;
  // Validate (first boot has garbage values)
  if (isnan(calibrationFactor) || calibrationFactor == 0.0f) calibrationFactor = 61.69f;
  if (isnan(offset))                                         offset = 0.0f;
  if (threshold <= 0 || threshold > 1000)                    threshold = 11;

  // ── WiFi Access Point ──
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  if (WiFi.softAP(WIFI_AP_SSID)) {
    Serial.println("WiFi AP: " + String(WIFI_AP_SSID));
    Serial.println("IP: " + WiFi.softAPIP().toString());
  } else {
    Serial.println("WiFi AP failed");
  }

  // ── Web server routes ──
  wifiServer.on("/", []() {
    wifiServer.send(200, "text/html",
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>"
        "body{font-family:sans-serif;text-align:center;padding:40px 20px;"
             "background:#111;color:#fff;margin:0;}"
        "h1{color:#D4930A;font-size:13px;letter-spacing:3px;text-transform:uppercase;margin-bottom:8px;}"
        "#w{font-size:80px;font-weight:900;margin:24px 0 8px;}"
        "#s{font-size:13px;color:#888;letter-spacing:1px;}"
      "</style></head>"
      "<body>"
        "<h1>Virtus Scale</h1>"
        "<div id='w'>--</div>"
        "<div id='s'>connecting...</div>"
        "<script>"
          "setInterval(()=>{"
            "fetch('/weight').then(r=>r.json()).then(d=>{"
              "document.getElementById('w').textContent=d.weight+' kg';"
              "document.getElementById('s').textContent=d.stable?'STABLE':'UNSTABLE';"
            "}).catch(()=>{});"
          "},500);"
        "</script>"
      "</body></html>");
  });

  wifiServer.on("/weight", []() {
    wifiServer.sendHeader("Access-Control-Allow-Origin", "*");
    long rW = adaptiveRound(currentWeight);
    long rT = adaptiveRound((float)totalUnloaded);
    char json[96];
    snprintf(json, sizeof(json),
      "{\"weight\":%ld,\"total\":%ld,\"stable\":%s,\"unloading\":%s}",
      rW, rT, stable ? "true" : "false", unloading ? "true" : "false");
    wifiServer.send(200, "application/json", json);
  });

  wifiServer.on("/status", []() {
    wifiServer.sendHeader("Access-Control-Allow-Origin", "*");
    // Include full device identity so the app can populate the Device Info card
    // when connected via WiFi remote mode (no BLE INFO packet available).
    uint64_t chipId = ESP.getEfuseMac();
    char serial[13];
    snprintf(serial, sizeof(serial), "%04X%08X",
             (uint16_t)(chipId >> 32), (uint32_t)chipId);
    float battV = readVoltage(PIN_BATT_ADC, BATT_DIVIDER_RATIO);
    float extV  = readVoltage(PIN_EXT_ADC,  EXT_DIVIDER_RATIO);
    int   pct   = voltageToBattPct(battV);
    char buf[256];
    snprintf(buf, sizeof(buf),
      "{\"primaryConnected\":%s,\"firmware\":\"%s\",\"model\":\"%s\","
      "\"serial\":\"%s\",\"batt\":%d,\"batt_v\":%.2f,\"ext_v\":%.1f}",
      deviceConnected ? "true" : "false",
      FIRMWARE_VERSION, DEVICE_MODEL, serial,
      pct, battV, extV);
    wifiServer.send(200, "application/json", String(buf));
  });

  wifiServer.begin();
  Serial.println("Web server ready — open 192.168.4.1 in browser");

  // ── BLE ──
  BLEDevice::init("VM#1");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pSvc = pServer->createService(SERVICE_UUID);

  pTxChar = pSvc->createCharacteristic(
    CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  pRxChar = pSvc->createCharacteristic(
    CHARACTERISTIC_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());

  pSvc->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising: VM#1");

  // ── HX711 ──
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibrationFactor);
  delay(500);
  if (scale.is_ready()) {
    lastValid = scale.get_units(1);
    Serial.println("HX711 ready");
  } else {
    Serial.println("HX711 not found — check wiring DT=27 SCK=26");
  }
}

// ── LOOP ───────────────────────────────────────────────────────
void loop() {
  // Handle deferred OTA restart (scheduled from BLE write callback).
  // This keeps ESP.restart() out of the BLE callback so the radio can
  // flush OTA_OK before the reboot.
  if (pendingRestart && (long)(millis() - pendingRestartAt) >= 0) {
    Serial.println("OTA restart firing from loop()");
    delay(50);
    ESP.restart();
  }

  wifiServer.handleClient();

  // BLE reconnection
  if (!deviceConnected && oldConnected) {
    delay(500);
    pServer->startAdvertising();
    oldConnected = false;
  }
  if (deviceConnected && !oldConnected) {
    oldConnected = true;
  }

  // Skip weight processing during OTA flash
  if (otaInProgress) return;

  // Read weight and subtract tare offset
  currentWeight = readHX711Weight() - offset;

  unsigned long now = millis();

  // ── Stability check ──
  if (now - lastCompareTime >= COMPARE_INTERVAL) {
    lastCompareTime = now;
    if (currentWeight > (float)(compareWeight - threshold) &&
        currentWeight < (float)(compareWeight + threshold)) {
      stable = true;
    } else {
      stable        = false;
      compareWeight = (int)currentWeight;
    }
  }

  // ── Unload tracking ──
  // Logic: startWeight(600) - currentWeight(-300) = 900kg unloaded
  if (unloading) {
    totalUnloaded = lastUnload + (unloadStartWeight - (int)currentWeight);
  } else {
    totalUnloaded = lastUnload;
  }

  // ── Send BLE weight packet every 500ms ──
  if (deviceConnected && now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    // 2kg resolution — fine enough for app auto-unload detection
    long rW = (long)(roundf(currentWeight / 2.0f) * 2.0f);
    long rT = adaptiveRound((float)totalUnloaded);
    String pkt = "P:" + String(rW)
               + ",L:" + String(rT)
               + ",S:" + String(stable    ? "1" : "0")
               + ",U:" + String(unloading ? "1" : "0");
    sendLine(pkt);
  }

  // ── Send battery/voltage status every 10s ──
  if (deviceConnected && now - lastStatusTime >= STATUS_INTERVAL) {
    lastStatusTime = now;
    sendStatus();
  }
}
