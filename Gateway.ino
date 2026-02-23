#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>

// ===== WiFi / Cloud =====
// ===== WiFi / 云端 =====
const char* WIFI_SSID = "moon";
const char* WIFI_PASS = "Hhj181520";
const char* API_URL   = "https://api-production-e339.up.railway.app/data";

static WiFiClientSecure s_client;

// ===== BLE (must match Sensor) =====
// ===== BLE（必须与 Sensor 端一致）=====
static NimBLEUUID SVC_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
static const char* SENSOR_NAME = "ELEV-SENSOR";

static NimBLEAdvertisedDevice* g_found = nullptr;
static NimBLEClient* g_client = nullptr;
static NimBLERemoteCharacteristic* g_chr = nullptr;

// 打印十六进制：方便排查 BLE 数据里是否夹杂不可见字符
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

// 从原始字节流里截取第一个 '{' 到最后一个 '}'，去掉“脏头脏尾”
// Slice JSON object from raw bytes: keep from first '{' to last '}', removing junk bytes
static String sliceJsonObject(const uint8_t* p, size_t n) {
  // 截取第一个 '{' 到最后一个 '}'，去掉脏头脏尾
  // Slice from first '{' to last '}' to remove prefix/suffix junk
  int first = -1, last = -1;
  for (size_t i = 0; i < n; i++) if (p[i] == '{') { first = (int)i; break; }
  for (int i = (int)n - 1; i >= 0; i--) if (p[i] == '}') { last = i; break; }
  if (first < 0 || last < 0 || last <= first) return String();

  String out;
  out.reserve((size_t)(last - first + 1));
  for (int i = first; i <= last; i++) out += (char)p[i];
  return out;
}

// 将 JSON 发送到 Railway API（HTTPS POST /data）
// POST JSON to Railway API (HTTPS POST /data)
bool postJsonToCloud(const String& json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GW] WiFi not connected, skip POST"); // WiFi 未连接，跳过 / WiFi not connected, skip
    return false;
  }

  // 调通阶段先不校验证书（生产建议换成 CA 校验）
  // Skip TLS cert validation during bring-up (use CA validation for production)
  s_client.setInsecure();
  s_client.setTimeout(15000);

  HTTPClient http;
  http.setTimeout(15000);

  if (!http.begin(s_client, API_URL)) {
    Serial.println("[GW] http.begin failed"); // 无法建立 HTTPS 请求 / Failed to init HTTPS request
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("Connection", "close");

  // 打印即将发送的 JSON（作为网关侧证据）
  // Print outgoing JSON (gateway-side evidence)
  Serial.print("[GW] POST body: ");
  Serial.println(json);

  int code = http.POST((uint8_t*)json.c_str(), json.length());
  String resp = http.getString();

  Serial.printf("[GW] POST /data code=%d\n", code); // HTTP 状态码 / HTTP status code
  Serial.print("[GW] resp: ");                      // 后端返回 / Backend response
  Serial.println(resp);

  http.end();
  return (code >= 200 && code < 300);
}

// 连接 WiFi：用于把网关转发的数据上传到云端
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

// BLE 扫描回调：按名字或 service UUID 找到 Sensor
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

// 开始扫描若干秒
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

// 连接到 Sensor，并拿到目标 characteristic（用于 readValue）
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

// 重置 BLE 状态：断开连接并清理对象
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

  // 先连 WiFi，再做 BLE（否则后面转发没网）
  // Connect WiFi first, then do BLE (cloud forwarding needs network)
  wifiConnect();

  // 初始化 BLE（作为 Central 扫描 + 连接）
  // Init BLE (as Central: scan + connect)
  NimBLEDevice::init("ELEV-GW");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // 启动扫描
  // Start scanning
  bleStartScan(5);
}

void loop() {
  // 还没连接：扫 -> 连
  // Not connected yet: scan -> connect
  if (!g_client || !g_client->isConnected() || !g_chr) {
    if (g_found) bleConnectAndGetChar();
    else bleStartScan(5);
    delay(200);
    return;
  }

  // 已连接：readValue 读取 Sensor 写入的数据
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

  // 打印原始内容（不可见字符用 '.' 替代）
  // Print raw content (replace non-printable chars with '.')
  Serial.printf("[GW] RAW(len=%u): ", (unsigned)n);
  for (size_t i = 0; i < n; i++) {
    char c = (char)bytes[i];
    if (c >= 32 && c <= 126) Serial.print(c);
    else Serial.print('.');
  }
  Serial.println();

  // 打印十六进制，进一步排查
  // Dump hex for deeper debugging
  dumpHex(bytes, n);

  // 清洗出 JSON 对象
  // Extract/clean JSON object
  String clean = sliceJsonObject(bytes, n);
  Serial.print("[GW] CLEAN: ");
  Serial.println(clean);

  if (clean.length() == 0) {
    Serial.println("[GW] CLEAN empty -> skip");
    delay(1000);
    return;
  }

  // 解析 Sensor 的 JSON
  // Parse Sensor JSON
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, clean);
  
  // doc 里应该来自 Sensor：pressure/accel/gyro/mag/moving
  // doc should come from Sensor: pressure/accel/gyro/mag/moving
  // 为了“队友后端最稳”，我们重新组一个 out，只发这 5 个字段（不夹杂 device_id 等）
  // For maximum backend compatibility, rebuild 'out' with only these 5 fields (no extra keys like device_id)
  StaticJsonDocument<256> out;

  // pressure (float)
  // pressure（浮点）
  if (!doc["pressure"].isNull()) out["pressure"] = doc["pressure"].as<float>();
  else out["pressure"] = 0.0; // 理论不该走到这 / should not happen

  // accel (int)
  // accel（整数）
  if (!doc["accel"].isNull()) out["accel"] = doc["accel"].as<int>();
  else out["accel"] = 0;

  // gyro (float)
  // gyro（浮点）
  if (!doc["gyro"].isNull()) out["gyro"] = doc["gyro"].as<float>();
  else out["gyro"] = 0.0;

  // mag (float)
  // mag（浮点）
  if (!doc["mag"].isNull()) out["mag"] = doc["mag"].as<float>();
  else out["mag"] = 0.0;

  // moving 必须是 0/1 int（不要 boolean）
  // moving must be 0/1 int (not boolean)
  int moving = 0;
  if (!doc["moving"].isNull()) {
    // 兼容：如果 Sensor 发的是 bool/字符串，也强制转成 0/1
    // Compatibility: if Sensor sends bool/string, force it to 0/1
    if (doc["moving"].is<bool>()) moving = doc["moving"].as<bool>() ? 1 : 0;
    else moving = doc["moving"].as<int>() ? 1 : 0;
  }
  out["moving"] = moving;

  // 输出最终要发给后端的 JSON
  // Final JSON to be sent to backend
  String jsonOut;
  serializeJson(out, jsonOut);
  Serial.print("[GW] JSON_OUT: ");
  Serial.println(jsonOut);

  // 转发到 Railway API
  // Forward to Railway API
  postJsonToCloud(jsonOut);

  delay(1000);
}