#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include "ICM_20948.h"
#include "dsplp_io.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <float.h>
#include <numeric>
#include <vector>

#include <NimBLEDevice.h>

// Temporary Part
#ifndef PRESSURE_SDA_PIN
#define PRESSURE_SDA_PIN 8   // TODO: replace with your actual SDA pin
#endif
#ifndef PRESSURE_SCL_PIN
#define PRESSURE_SCL_PIN 9   // TODO: replace with your actual SCL pin
#endif


// ---------- Globals ----------
// ---------- Data types ----------

struct imu_data {
  float accel_data_x;
  float accel_data_y;
  float accel_data_z;

  float mag_data_x;
  float mag_data_y;
  float mag_data_z;

  // Keep gyro fields commented for future extension
  // float gyro_data_x;
  // float gyro_data_y;
  // float gyro_data_z;
};

struct sensor_data {
  float pressure_data;

  float accel_data_x;
  float accel_data_y;
  float accel_data_z;

  float mag_data_x;
  float mag_data_y;
  float mag_data_z;

  // Keep gyro fields commented for future extension
  // float gyro_data_x;
  // float gyro_data_y;
  // float gyro_data_z;

  bool moving;
  int floor;
  bool door_open;
};

// assume that door is open when the program starts
bool door_is_open = true;
QueueHandle_t data_queue;

ICM_20948_I2C myICM;
BMP581 pressureSensor;
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47



// =========================
// BLE setting (Sensor -> Gateway)
// =========================

// BLE identity (must match Gateway)
static const char* DEVICE_NAME = "ELEV-SENSOR";
static NimBLEUUID SVC_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

static NimBLEServer* server = nullptr;
static NimBLECharacteristic* chr = nullptr;

// Optional connection callbacks (debug)
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) {
    Serial.println("[SENSOR] BLE client connected");
  }
  void onDisconnect(NimBLEServer*) {
    Serial.println("[SENSOR] BLE client disconnected -> restart advertising");
    NimBLEDevice::startAdvertising();
  }
};

// BLE module
void bleSetup() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(SVC_UUID);
  chr = svc->createCharacteristic(CHR_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  chr->createDescriptor("2902");
  chr->setValue("{\"boot\":true}");
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->start();

  Serial.println("[SENSOR] BLE advertising started");
}

// MODULE 2: send ONE JSON object to Gateway (single sample per notify)
bool sendPackageToGateway(const char* json, size_t len) {
  if (!chr) return false;
  if (server && server->getConnectedCount() == 0) return false;

  chr->setValue((uint8_t*)json, len);
  chr->notify();

  Serial.print("[SENSOR] BLE notify: ");
  Serial.println(json);
  return true;
}

// =========================
// Data (max retention)
// =========================

float get_pressure() {
  bmp5_sensor_data data = {0};
  int8_t err = pressureSensor.getSensorData(&data);

  if (err == BMP5_OK) {
    return data.pressure;
  } else {
    Serial.print("Error getting data from sensor! Error code: ");
    Serial.println(err);
    return NAN;
  }
}

imu_data get_imu_data() {
  imu_data data;

  if (myICM.dataReady()) {
    myICM.getAGMT();
  } else {
    Serial.println("Error getting data from sensor!");
    return data;
  }

  data.accel_data_x = myICM.accX();
  data.accel_data_y = myICM.accY();
  data.accel_data_z = myICM.accZ();

  data.mag_data_x = myICM.magX();
  data.mag_data_y = myICM.magY();
  data.mag_data_z = myICM.magZ();

  // Keep gyro reads commented for future use
  // data.gyro_data_x = myICM.gyrX();
  // data.gyro_data_y = myICM.gyrY();
  // data.gyro_data_z = myICM.gyrZ();

  return data;
}

// Calculate if the pressure is changing fast
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

// Calculate if the magnetic signal is changing fast
bool magnetic_change(float mu, imu_data imu, float gamma, float sigma_2) {
  float change_delta = imu.mag_data_z - mu;
  float T = (change_delta * change_delta) / sigma_2;
  return (T > gamma);
}

// Keep your alternative magnetic-change draft commented
// float magnetic_change(float magnetic_z_mu, imu_data imu, float gamma, float magnetic_sigma_2) {
//     //float change_delta_z = imu.mag_data_z - magnetic_z_mu;
//     float change = imu.mag_data_z*imu.mag_data_z+imu.mag_data_z*imu.mag_data_z+imu.mag_data_z
//     float T = (change_delta_z*change_delta_z) / magnetic_sigma_2;
//     return T;
// }

// Debug output
void output_data(float pressure, imu_data imu, bool moving_now, int current_floor, bool door_open) {
  Serial.println();
  Serial.println("========== SENSOR STATUS ==========");

  Serial.print("moving: ");
  Serial.println(moving_now ? "true" : "false");

  Serial.print("floor: ");
  Serial.println(current_floor);

  Serial.print("pressure: ");
  Serial.println(pressure, 3);

  Serial.print("accel (x,y,z): ");
  Serial.print(imu.accel_data_x, 3); Serial.print(", ");
  Serial.print(imu.accel_data_y, 3); Serial.print(", ");
  Serial.println(imu.accel_data_z, 3);

  Serial.print("mag   (x,y,z): ");
  Serial.print(imu.mag_data_x, 3); Serial.print(", ");
  Serial.print(imu.mag_data_y, 3); Serial.print(", ");
  Serial.println(imu.mag_data_z, 3);

  Serial.print("door_open: ");
  Serial.println(door_open ? "true" : "false");

  Serial.println("==================================");
}

// Task 1: Sensor acquisition + ML + queue push
void sensorTask(void *parameter) {
  delay(10);

  float pressure_mu = get_pressure();
  base = get_pressure();
  float gamma = 8.8075;
  float pressure_sigma_2 = 30;
  int N_MAX = 4;
  float pressure_delta;

  imu_data imu = get_imu_data();

  float magnetic_mu = std::sqrt(imu.mag_data_x * imu.mag_data_x +
                                imu.mag_data_y * imu.mag_data_y +
                                imu.mag_data_z * imu.mag_data_z);

  float magnetic_sigma_2 = 4;
  float magnetic_delta;

  int count = 0;
  int door_cooldown = 0;
  int test = 0;

  while (true) {
    float pressure = get_pressure();
    imu_data imu = get_imu_data();

    bool moving_now = pressure_change(pressure_mu, pressure, gamma, pressure_sigma_2);
    int current_floor = classify_floor(pressure);

    float magn = std::sqrt(imu.mag_data_x * imu.mag_data_x +
                           imu.mag_data_y * imu.mag_data_y +
                           imu.mag_data_z * imu.mag_data_z);

    magnetic_delta = 1.0f / 100;
    magnetic_mu = (1 - magnetic_delta) * magnetic_mu + magnetic_delta * magn;
    float diff = magnetic_mu - magn;
    float temp = diff * diff / magnetic_sigma_2;
    bool door_moving = temp > gamma;
    test += door_moving;

    if (count > 19) { // ~1 second (20 * 50ms)
      count = 0;

      if (door_cooldown > 0) door_cooldown--;

      if (test > 2 && !moving_now && door_cooldown < 1) {
        door_is_open = !door_is_open;
        door_cooldown = 9;
      }
      if (moving_now) {
        door_is_open = false;
      }

      output_data(pressure, imu, moving_now, current_floor, door_is_open);
      Serial.printf(" T: "); Serial.print(test);
      test = 0;

      sensor_data data;
      data.pressure_data = pressure;

      data.accel_data_x  = imu.accel_data_x;
      data.accel_data_y  = imu.accel_data_y;
      data.accel_data_z  = imu.accel_data_z;

      data.mag_data_x    = imu.mag_data_x;
      data.mag_data_y    = imu.mag_data_y;
      data.mag_data_z    = imu.mag_data_z;

      data.moving        = moving_now;
      data.floor         = current_floor;
      data.door_open     = door_is_open;

      // Keep gyro assignment commented for future
      // data.gyro_data_x = imu.gyro_data_x;
      // data.gyro_data_y = imu.gyro_data_y;
      // data.gyro_data_z = imu.gyro_data_z;

      xQueueSend(data_queue, &data, portMAX_DELAY);

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

// Task 2: BLE sender task (sends one JSON object per notify)
// Output schema (sent over BLE) includes:
// pressure, accel{x,y,z}, gyro{x,y,z}, mag{x,y,z}, moving(0/1), floor, door_open(0/1)
static bool build_single_json(const sensor_data& d, char* out, size_t out_sz) {
  // Note: gyro is currently placeholder 0/0/0 (kept consistent with your previous code)
  int n = snprintf(out, out_sz,
    "{"
      "\"pressure\":%.3f,"
      "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"mag\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"moving\":%d,"
      "\"floor\":%d,"
      "\"door_open\":%d"
    "}",
    d.pressure_data,
    d.accel_data_x, d.accel_data_y, d.accel_data_z,
    0.0f, 0.0f, 0.0f,               // gyro placeholder
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
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
    }

    if (!build_single_json(pending, jsonBuffer, sizeof(jsonBuffer))) {
      Serial.println("[SENSOR] JSON build failed (buffer too small?)");
      has_pending = false;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    bool ok = sendPackageToGateway(jsonBuffer, strlen(jsonBuffer));
    if (ok) {
      has_pending = false;
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

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

  xTaskCreatePinnedToCore(
    sensorTask,
    "SensorTask",
    8192,
    NULL,
    2,
    NULL,
    1);

  xTaskCreatePinnedToCore(
    bleSenderTask,
    "BLESenderTask",
    8192,
    NULL,
    1,
    NULL,
    0);

  delay(500);
}

void loop() {
  // Empty: work is done in FreeRTOS tasks
}