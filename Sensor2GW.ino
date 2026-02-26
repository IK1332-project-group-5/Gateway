#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include "ICM_20948.h"
#include "dsplp_io.h"

#include <ArduinoJson.h>
#include <math.h>
#include <float.h>

// =========================
// BLE (Sensor -> Gateway)
// =========================
#include <NimBLEDevice.h>

// =====================================================
// Data structures (MUST be before any function definitions)
// =====================================================
bool door_is_open = true;

struct imu_data {
  float accel_data_x;
  float accel_data_y;
  float accel_data_z;
  float mag_data_x;
  float mag_data_y;
  float mag_data_z;
};

struct sensor_data {
  float pressure_data;
  float accel_data_x;
  float accel_data_y;
  float accel_data_z;
  float mag_data_x;
  float mag_data_y;
  float mag_data_z;
  bool moving;
  int floor;
  bool door_open;
};

// =====================================================
// Forward declarations
// =====================================================
static void startAdv();
void bleSetup();
static bool sendPackageToGateway(const char* json, size_t len);

float get_pressure();
imu_data get_imu_data();

bool pressure_change(float mu, float pressure, float gamma, float sigma_2);
int  classify_floor(double current);
bool magnetic_change(float mu, float magn, float gamma, float sigma_2);

void output_data(float pressure, imu_data imu, bool moving_now, int current_floor, bool door_open);

void sensorTask(void *parameter);
void bleSenderTask(void *parameter);
void advWatchdogTask(void *parameter);

static bool build_single_json(const sensor_data& d, char* out, size_t out_sz);

// =====================================================
// BLE identity (must match Gateway)
// =====================================================
static const char* DEVICE_NAME = "ELEV-SENSOR";
static NimBLEUUID SVC_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

static NimBLEServer* server = nullptr;
static NimBLECharacteristic* chr = nullptr;

// Stronger advertising restart support
static NimBLEAdvertising* g_adv = nullptr;

// =====================================================
// Queue & Sensors
// =====================================================
QueueHandle_t data_queue;

ICM_20948_I2C myICM;
BMP581 pressureSensor;
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // usually 0x47

// =====================================================
// BLE callbacks
// =====================================================
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) {
    Serial.println("[SENSOR] BLE client connected");
  }
  void onDisconnect(NimBLEServer*) {
    Serial.println("[SENSOR] BLE client disconnected -> restart advertising");
    startAdv();  // best-effort restart, watchdog will also kick
  }
};

// =====================================================
// Advertising helper (compatible with your NimBLE library)
// =====================================================
static void startAdv() {
  if (!g_adv) g_adv = NimBLEDevice::getAdvertising();

  // Hard restart advertising (works better than startAdvertising() on some stacks)
  g_adv->stop();
  delay(30);

  NimBLEAdvertisementData advData;
  advData.setName(DEVICE_NAME);
  advData.addServiceUUID(SVC_UUID);

  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);

  g_adv->setAdvertisementData(advData);
  g_adv->setScanResponseData(scanData);

  g_adv->start();
  Serial.println("[SENSOR] Advertising (re)started");
}

// =====================================================
// MODULE 1: BLE setup
// =====================================================
void bleSetup() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Keep BLE simple (avoid pairing/security issues)
  NimBLEDevice::setSecurityAuth(false, false, false);

  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(SVC_UUID);
  chr = svc->createCharacteristic(CHR_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  chr->createDescriptor("2902");

  chr->setValue("{\"boot\":true}");
  svc->start();

  g_adv = NimBLEDevice::getAdvertising();
  startAdv();

  Serial.println("[SENSOR] BLE setup complete");
}

// =====================================================
// MODULE 2: send one JSON object to Gateway via notify
// =====================================================
static bool sendPackageToGateway(const char* json, size_t len) {
  if (!chr) return false;
  if (!server) return false;
  if (server->getConnectedCount() == 0) return false;

  chr->setValue((uint8_t*)json, len);
  chr->notify();

  Serial.println();
  Serial.println("---------- BLE PAYLOAD ----------");
  Serial.println(json);
  Serial.println("--------------------------------");
  return true;
}

// =====================================================
// Sensor reading helpers
// =====================================================
float get_pressure() {
  bmp5_sensor_data data = {0};
  int8_t err = pressureSensor.getSensorData(&data);
  if (err == BMP5_OK) return data.pressure;

  Serial.print("Error getting pressure! code=");
  Serial.println(err);
  return NAN;
}

imu_data get_imu_data() {
  imu_data data;

  if (myICM.dataReady()) myICM.getAGMT();
  else {
    Serial.println("Error getting IMU data!");
    return data;
  }

  data.accel_data_x = myICM.accX();
  data.accel_data_y = myICM.accY();
  data.accel_data_z = myICM.accZ();

  data.mag_data_x = myICM.magX();
  data.mag_data_y = myICM.magY();
  data.mag_data_z = myICM.magZ();

  return data;
}

// =====================================================
// Logic / ML
// =====================================================
bool pressure_change(float mu, float pressure, float gamma, float sigma_2) {
  float change_delta = pressure - mu;
  float T = (change_delta * change_delta) / sigma_2;
  return (T > gamma);
}

#define NUM_LEVELS 6
double floor_offsets[NUM_LEVELS] = {
  0.0,   // floor 2
  35.0,  // floor 3
  70.0,  // floor 4
  105.0, // floor 5
  140.0, // floor 6
  175.0  // floor 7
};
double base = 0;

int classify_floor(double current) {
  double delta = base - current;
  int best_floor = 0;
  double min_error = DBL_MAX;

  for (int i = 0; i < NUM_LEVELS; i++) {
    double error = fabs(delta - floor_offsets[i]);
    if (error < min_error) {
      min_error = error;
      best_floor = i;
    }
  }
  return best_floor + 2;
}

bool magnetic_change(float mu, float magn, float gamma, float sigma_2) {
  float diff = mu - magn;
  float temp = diff * diff / sigma_2;
  return temp > gamma;
}

// =====================================================
// Debug output (readable)
// =====================================================
void output_data(float pressure, imu_data imu, bool moving_now, int current_floor, bool door_open) {
  Serial.println();
  Serial.println("========== SENSOR STATUS ==========");
  Serial.print("moving: "); Serial.println(moving_now ? "true" : "false");
  Serial.print("floor: "); Serial.println(current_floor);
  Serial.print("pressure: "); Serial.println(pressure, 3);

  Serial.print("accel (x,y,z): ");
  Serial.print(imu.accel_data_x, 3); Serial.print(", ");
  Serial.print(imu.accel_data_y, 3); Serial.print(", ");
  Serial.println(imu.accel_data_z, 3);

  Serial.print("mag   (x,y,z): ");
  Serial.print(imu.mag_data_x, 3); Serial.print(", ");
  Serial.print(imu.mag_data_y, 3); Serial.print(", ");
  Serial.println(imu.mag_data_z, 3);

  Serial.print("door_open: "); Serial.println(door_open ? "true" : "false");
  Serial.println("==================================");
}

// =====================================================
// Task 1: acquisition + inference + queue
// =====================================================
void sensorTask(void *parameter) {
  delay(10);

  float pressure_mu = get_pressure();
  base = get_pressure();

  float gamma = 8.8075;
  float pressure_sigma_2 = 30;
  int N_MAX = 4;
  float pressure_delta;

  imu_data imu0 = get_imu_data();
  float magnetic_mu = sqrt(imu0.mag_data_x * imu0.mag_data_x +
                           imu0.mag_data_y * imu0.mag_data_y +
                           imu0.mag_data_z * imu0.mag_data_z);

  float magnetic_sigma_2 = 4;
  float magnetic_delta = 1.0f / 100;

  int count = 0;
  int door_cooldown = 0;
  int door_count = 0;

  while (true) {
    float pressure = get_pressure();
    imu_data imu = get_imu_data();

    bool moving_now = pressure_change(pressure_mu, pressure, gamma, pressure_sigma_2);
    int current_floor = classify_floor(pressure);

    float magn = sqrt(imu.mag_data_x * imu.mag_data_x +
                      imu.mag_data_y * imu.mag_data_y +
                      imu.mag_data_z * imu.mag_data_z);

    door_count += magnetic_change(magnetic_mu, magn, gamma, magnetic_sigma_2);
    magnetic_mu = (1 - magnetic_delta) * magnetic_mu + magnetic_delta * magn;

    if (count > 19) {
      count = 0;

      if (door_cooldown > 0) door_cooldown--;

      if (door_count > 2 && !moving_now && door_cooldown < 1) {
        door_is_open = !door_is_open;
        door_cooldown = 9;
      }
      if (moving_now) {
        door_is_open = false;
      }

      output_data(pressure, imu, moving_now, current_floor, door_is_open);
      door_count = 0;

      sensor_data d;
      d.pressure_data = pressure;
      d.accel_data_x  = imu.accel_data_x;
      d.accel_data_y  = imu.accel_data_y;
      d.accel_data_z  = imu.accel_data_z;
      d.mag_data_x    = imu.mag_data_x;
      d.mag_data_y    = imu.mag_data_y;
      d.mag_data_z    = imu.mag_data_z;
      d.moving        = moving_now;
      d.floor         = current_floor;
      d.door_open     = door_is_open;

      xQueueSend(data_queue, &d, portMAX_DELAY);

      pressure_delta = 1.0f / N_MAX;
      pressure_mu = (1 - pressure_delta) * pressure_mu + pressure_delta * pressure;

      if (current_floor == 2) {
        base = base - (base - pressure) / N_MAX;
      }
    }

    count++;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =====================================================
// Task 2: queue -> JSON -> BLE notify
// =====================================================
static bool build_single_json(const sensor_data& d, char* out, size_t out_sz) {
  int n = snprintf(out, out_sz,
    "{"
      "\"pressure\":%.3f,"
      "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"mag\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"moving\":%d,"
      "\"floor\":%d,"
      "\"door_open\":%d"
    "}",
    d.pressure_data,
    d.accel_data_x, d.accel_data_y, d.accel_data_z,
    d.mag_data_x,   d.mag_data_y,   d.mag_data_z,
    d.moving ? 1 : 0,
    d.floor,
    d.door_open ? 1 : 0
  );
  return (n > 0 && (size_t)n < out_sz);
}

void bleSenderTask(void *parameter) {
  sensor_data pending{};
  bool has_pending = false;
  static char jsonBuffer[512];

  while (true) {
    if (!server || server->getConnectedCount() == 0 || !chr) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!has_pending) {
      if (uxQueueMessagesWaiting(data_queue) > 0) {
        xQueueReceive(data_queue, &pending, 0);
        has_pending = true;
      } else {
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }
    }

    if (!build_single_json(pending, jsonBuffer, sizeof(jsonBuffer))) {
      Serial.println("[SENSOR] JSON build failed (buffer too small?)");
      has_pending = false;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    bool ok = sendPackageToGateway(jsonBuffer, strlen(jsonBuffer));
    if (ok) has_pending = false;

    // 1Hz is enough and improves stability in weak-signal environments
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// NEW: Advertising watchdog
// If link breaks without clean disconnect, this keeps advertising alive
// =====================================================
void advWatchdogTask(void *parameter) {
  while (true) {
    if (server && server->getConnectedCount() == 0) {
      // Force a periodic advertising restart when not connected
      startAdv();
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // every 2 seconds
  }
}

// =====================================================
// setup / loop
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(PRESSURE_SDA_PIN, PRESSURE_SCL_PIN);

  while (pressureSensor.beginI2C(i2cAddress) != BMP5_OK) {
    Serial.println("Error: BMP581 not connected, check wiring and I2C address!");
    delay(1000);
  }

  while (myICM.begin(Wire, 1) != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 not detected!");
    delay(1000);
  }

  pinMode(LED_SDA_IO, OUTPUT);
  pinMode(LED_SHCP_IO, OUTPUT);
  pinMode(LED_STCP_IO, OUTPUT);

  data_queue = xQueueCreate(1000, sizeof(sensor_data));

  bleSetup();

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(bleSenderTask, "BLESenderTask", 8192, NULL, 1, NULL, 0);

  // Low priority watchdog to keep advertising alive
  xTaskCreatePinnedToCore(advWatchdogTask, "AdvWatchdog", 4096, NULL, 0, NULL, 0);

  delay(500);
}

void loop() {
  // Work is done in FreeRTOS tasks
}