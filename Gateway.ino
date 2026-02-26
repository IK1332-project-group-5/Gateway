#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>

// ===== WiFi / Cloud =====
const char* WIFI_SSID = "moon";
const char* WIFI_PASS = "Hhj181520";
const char* API_URL   = "https://api-production-4aeb.up.railway.app/data";

// ===== API KEY (required by backend) =====
const char* API_KEY   = "6rvhhBwNKo8wigI33v7fufTT6pIMN5iU";

static WiFiClientSecure s_client;

// ===== BLE (must match Sensor) =====
static NimBLEUUID SVC_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
static const char* SENSOR_NAME = "ELEV-SENSOR";

static NimBLEAdvertisedDevice* g_found = nullptr;
static NimBLEClient* g_client = nullptr;
static NimBLERemoteCharacteristic* g_chr = nullptr;

// Dump hex: useful to debug invisible/extra bytes in BLE payload
static void dumpHex(const uint8_t* p, size_t n) {
  Serial.print("[GW] HEX: ");
  for (size_t i = 0; i < n; i++) {
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// Slice JSON object from raw bytes: keep from first '{' to last '}', removing junk bytes
static String sliceJsonObject(const uint8_t* p, size_t n) {
  int first = -1, last = -1;
  for (size_t i = 0; i < n; i++) if (p[i] == '{') { first = (int)i; break; }
  for (int i = (int)n - 1; i >= 0; i--) if (p[i] == '}') { last = i; break; }
  if (first < 0 || last < 0 || last <= first) return String();

  String out;
  out.reserve((size_t)(last - first + 1));
  for (int i = first; i <= last; i++) out += (char)p[i];
  return out;
}

// Build POST body in the SAME style as the original sensor uploader:
// Wrap a single object into an array: [ { ... } ]
// Add api_key into the object, and remove gyro to match backend schema.
static bool buildPostArrayFromSensorJson(const String& clean, String& outArrayJson) {
  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, clean);
  if (err) {
    Serial.print("[GW] JSON parse failed: ");
    Serial.println(err.c_str());
    return false;
  }

  // Inject required api_key (backend requirement)
  doc["api_key"] = API_KEY;

  // Remove gyro because backend does not need it (based on your backend example)
  doc.remove("gyro");

  // Wrap into array
  StaticJsonDocument<896> arr;
  JsonArray a = arr.to<JsonArray>();
  a.add(doc.as<JsonObject>());

  outArrayJson.clear();
  serializeJson(arr, outArrayJson);
  return true;
}

// POST JSON to Railway API (HTTPS POST /data)
bool postJsonToCloud(const String& json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GW] WiFi not connected, skip POST");
    return false;
  }

  // Skip TLS cert validation during bring-up (use CA validation for production)
  s_client.setInsecure();
  s_client.setTimeout(15000);

  HTTPClient http;
  http.setTimeout(15000);

  if (!http.begin(s_client, API_URL)) {
    Serial.println("[GW] http.begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("Connection", "close");

  Serial.print("[GW] POST body: ");
  Serial.println(json);

  int code = http.POST((uint8_t*)json.c_str(), json.length());
  String resp = http.getString();

  Serial.printf("[GW] POST /data code=%d\n", code);
  Serial.print("[GW] resp: ");
  Serial.println(resp);

  http.end();
  return (code >= 200 && code < 300);
}

// Connect WiFi: required for forwarding data to cloud
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[GW] WiFi connecting");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[GW] WiFi OK, IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[GW] WiFi connect FAILED");
  }
}

// BLE scan callback: find Sensor by name or service UUID
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) {
    if (adv->haveName() && adv->getName() == std::string(SENSOR_NAME)) {
      Serial.print("[GW] Found sensor by name: ");
      Serial.println(adv->getName().c_str());
      g_found = new NimBLEAdvertisedDevice(*adv);
      NimBLEDevice::getScan()->stop();
      return;
    }
    if (adv->isAdvertisingService(SVC_UUID)) {
      Serial.println("[GW] Found sensor by service UUID");
      g_found = new NimBLEAdvertisedDevice(*adv);
      NimBLEDevice::getScan()->stop();
      return;
    }
  }
};
static ScanCallbacks g_scanCb;

// Start scanning for a given number of seconds
void bleStartScan(uint32_t seconds = 5) {
  Serial.printf("[GW] BLE scanning (%us)...\n", seconds);
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&g_scanCb, false);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->setActiveScan(true);
  scan->start(seconds, false);
}

// Connect to Sensor and obtain the target characteristic (for readValue)
bool bleConnectAndGetChar() {
  if (!g_found) return false;

  Serial.println("[GW] Connecting to sensor...");
  g_client = NimBLEDevice::createClient();

  if (!g_client->connect(g_found)) {
    Serial.println("[GW] BLE connect FAILED");
    NimBLEDevice::deleteClient(g_client);
    g_client = nullptr;
    delete g_found; g_found = nullptr;
    return false;
  }
  Serial.println("[GW] BLE connected");

  NimBLERemoteService* svc = g_client->getService(SVC_UUID);
  if (!svc) {
    Serial.println("[GW] Service not found");
    g_client->disconnect();
    NimBLEDevice::deleteClient(g_client);
    g_client = nullptr;
    delete g_found; g_found = nullptr;
    return false;
  }

  g_chr = svc->getCharacteristic(CHR_UUID);
  if (!g_chr) {
    Serial.println("[GW] Characteristic not found");
    g_client->disconnect();
    NimBLEDevice::deleteClient(g_client);
    g_client = nullptr;
    delete g_found; g_found = nullptr;
    return false;
  }

  Serial.println("[GW] Ready to readValue()");
  delete g_found; g_found = nullptr;
  return true;
}

// Reset BLE state: disconnect and cleanup
void bleReset() {
  if (g_client) {
    if (g_client->isConnected()) g_client->disconnect();
    NimBLEDevice::deleteClient(g_client);
  }
  g_client = nullptr;
  g_chr = nullptr;
  if (g_found) { delete g_found; g_found = nullptr; }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[GW] boot");

  wifiConnect();

  NimBLEDevice::init("ELEV-GW");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  bleStartScan(5);
}

void loop() {
  // Not connected yet: scan -> connect
  if (!g_client || !g_client->isConnected() || !g_chr) {
    if (g_found) bleConnectAndGetChar();
    else bleStartScan(5);
    delay(200);
    return;
  }

  // Connected: read value written by Sensor
  std::string v;
  try { v = g_chr->readValue(); }
  catch (...) {
    Serial.println("[GW] readValue exception -> reset BLE");
    bleReset();
    delay(500);
    return;
  }

  if (v.empty()) {
    Serial.println("[GW] BLE read empty");
    delay(1000);
    return;
  }

  const uint8_t* bytes = (const uint8_t*)v.data();
  size_t n = v.size();

  // Print raw payload for debugging
  Serial.printf("[GW] RAW(len=%u): ", (unsigned)n);
  for (size_t i = 0; i < n; i++) {
    char c = (char)bytes[i];
    if (c >= 32 && c <= 126) Serial.print(c);
    else Serial.print('.');
  }
  Serial.println();

  dumpHex(bytes, n);

  // Clean JSON object
  String clean = sliceJsonObject(bytes, n);
  Serial.print("[GW] CLEAN: ");
  Serial.println(clean);

  if (clean.length() == 0) {
    Serial.println("[GW] CLEAN empty -> skip");
    delay(1000);
    return;
  }

  // Build POST body as JSON array (original uploader style) + api_key
  String postBody;
  if (!buildPostArrayFromSensorJson(clean, postBody)) {
    Serial.println("[GW] buildPostArrayFromSensorJson failed");
    delay(1000);
    return;
  }

  Serial.print("[GW] POST_ARRAY: ");
  Serial.println(postBody);

  // Forward to Railway
  postJsonToCloud(postBody);

  delay(1000);
}