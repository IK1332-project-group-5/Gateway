#include <NimBLEDevice.h>

// =====================================================
// CONFIG (must match Gateway)
// =====================================================
static const char* DEVICE_NAME = "ELEV-SENSOR";
static NimBLEUUID SVC_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

// =====================================================
// STATE
// =====================================================
static NimBLEServer* server = nullptr;
static NimBLECharacteristic* chr = nullptr;

// Connection callbacks (optional, for debugging)
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) override {
    Serial.println("[SENSOR] Client connected");
  }
  void onDisconnect(NimBLEServer*) override {
    Serial.println("[SENSOR] Client disconnected -> restart advertising");
    NimBLEDevice::startAdvertising();
  }
};

// =====================================================
// MODULE 1: BLE setup
// =====================================================
void bleSetup() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(SVC_UUID);
  chr = svc->createCharacteristic(CHR_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  chr->createDescriptor("2902");              // needed by many clients to subscribe notify
  chr->setValue("{\"boot\":true}");           // debug initial value
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->start();

  Serial.println("[SENSOR] Advertising started");
}

// =====================================================
// MODULE 2: build package + notify to Gateway
// =====================================================
void sendPackage(float pressure, int accel, float gyro, float mag, int moving01) {
  if (!chr) return;

  // Payload schema must match Gateway parser: pressure/accel/gyro/mag/moving
  // NOTE: moving uses 0/1 for compatibility (Gateway can convert to boolean)
  String payload = "{";
  payload += "\"pressure\":" + String(pressure, 1) + ",";
  payload += "\"accel\":"    + String(accel) + ",";
  payload += "\"gyro\":"     + String(gyro, 2) + ",";
  payload += "\"mag\":"      + String(mag, 2) + ",";
  payload += "\"moving\":"   + String(moving01);
  payload += "}";

  chr->setValue(payload.c_str());
  chr->notify();

  Serial.print("[SENSOR] notify: ");
  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== SENSOR BOOT ===");
  bleSetup();
}

void loop() {
  // TODO: replace with real sensor readings
  float pressure = 1012.0f;
  int   accel    = 4;
  float gyro     = 0.2f;
  float mag      = 10.0f;
  int   moving01 = 0; // 0/1

  sendPackage(pressure, accel, gyro, mag, moving01);
  delay(1000);
}