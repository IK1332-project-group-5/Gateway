#include <NimBLEDevice.h>

// 设备名（Gateway 用这个名字去扫）
// Device name (Gateway scans for this name)
static const char* DEVICE_NAME = "ELEV-SENSOR";

// 两边必须一致的 UUID（Nordic UART Service）
// UUIDs must match on both sides (Nordic UART Service)
static NimBLEUUID SVC_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

NimBLEServer* server = nullptr;
NimBLECharacteristic* chr = nullptr;
uint32_t counter = 0;

// 连接/断开回调：用于在串口里提示是否有客户端连接
// Connect/Disconnect callbacks: print connection status to Serial Monitor
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) {
    Serial.println("[SENSOR] Client connected"); // 客户端已连接 / Client connected
  }
  void onDisconnect(NimBLEServer*) {
    Serial.println("[SENSOR] Client disconnected"); // 客户端已断开 / Client disconnected
    NimBLEDevice::startAdvertising(); // 重新开始广播 / Restart advertising
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== SENSOR BOOT ==="); // 传感器启动 / Sensor boot

  // 初始化 BLE 设备并设置发射功率
  // Initialize BLE device and set TX power
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // 创建 GATT Server 并注册回调
  // Create GATT server and register callbacks
  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  // 创建服务与特征（支持 READ + NOTIFY）
  // Create service and characteristic (READ + NOTIFY)
  NimBLEService* svc = server->createService(SVC_UUID);
  chr = svc->createCharacteristic(CHR_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

  // 2902 描述符：很多客户端订阅 notify 时需要它
  // 2902 descriptor: many clients need it to subscribe to notifications
  chr->createDescriptor("2902");

  // 初始值：方便调试连接是否通
  // Initial value: handy for debugging connectivity
  chr->setValue("{\"boot\":true}");

  // 启动服务
  // Start service
  svc->start();

  // 开始广播（包含 service UUID，便于 Gateway 按 UUID 匹配）
  // Start advertising (include service UUID so Gateway can match by UUID)
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->start();
  Serial.println("[SENSOR] Advertising started"); // 已开始广播 / Advertising started
}

void loop() {
  // 你原来的模拟逻辑：楼层影响 pressure，accel 用计数
  // Your original simulation logic: floor affects pressure; accel is based on counter
  int floor = (counter % 6) + 1;
  float pressure = 101800.0f - (float)(floor * 25);
  int accel = (counter % 5);

  // 下面三个是“补齐后端要求字段”的模拟值
  // The following are simulated values to satisfy backend-required fields
  // gyro / mag 你可以理解成 IMU 的某个标量（比如模长），先用 float 模拟
  // gyro / mag can be treated as some IMU scalar (e.g., magnitude); simulated as float for now
  float gyro = 0.10f * (counter % 50);   // 0.0 ~ 4.9
  float mag  = 0.20f * (counter % 50);   // 0.0 ~ 9.8

  // moving 必须 0/1（int），别用 true/false
  // moving must be 0/1 (int), do not use true/false
  int moving = (accel >= 2) ? 1 : 0;

  // ====== 关键：payload 必须包含 pressure/accel/gyro/mag/moving ======
  // ====== Key: payload must include pressure/accel/gyro/mag/moving ======
  String payload = "{";
  payload += "\"pressure\":" + String(pressure, 1) + ",";
  payload += "\"accel\":" + String(accel) + ",";
  payload += "\"gyro\":" + String(gyro, 2) + ",";
  payload += "\"mag\":" + String(mag, 2) + ",";
  payload += "\"moving\":" + String(moving);
  payload += "}";

  // 写入特征值并通过 notify 推送给 Gateway
  // Update characteristic value and notify Gateway
  chr->setValue(payload.c_str());
  chr->notify();

  // 串口打印：作为“Sensor 端证据”
  // Serial print: evidence on Sensor side
  Serial.print("[SENSOR] notify: ");
  Serial.println(payload);

  counter++;
  delay(1000); // 1Hz 发送 / send at 1Hz
}