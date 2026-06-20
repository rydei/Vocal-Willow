#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <math.h>
#include <string>
#include "esp_bt.h"

// FieldSense ESP32 firmware
// ESP32 + MPU6050 bat-mounted hit/miss detector with low-power BLE events.

namespace {

constexpr char DEVICE_NAME[] = "FieldSense-Bat";
constexpr char FIRMWARE_VERSION[] = "0.1.0";

// Wiring defaults for common ESP32 DevKit boards.
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// Battery voltage divider: BAT+ -> R1 -> ADC pin -> R2 -> GND.
// Good starting values for a 1S2P 18650 pack: R1=220k, R2=100k, plus 100 nF
// from ADC pin to GND. Adjust BATTERY_CALIBRATION after measuring with a meter.
constexpr uint8_t BATTERY_ADC_PIN = 34;
constexpr float BATTERY_R1_OHMS = 220000.0f;
constexpr float BATTERY_R2_OHMS = 100000.0f;
constexpr float BATTERY_CALIBRATION = 1.000f;
constexpr uint32_t BATTERY_UPDATE_MS = 10000;

// MPU6050 constants.
constexpr uint8_t MPU_ADDR_LOW = 0x68;
constexpr uint8_t MPU_ADDR_HIGH = 0x69;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr float ACCEL_LSB_PER_G = 2048.0f;   // MPU6050 +/-16 g
constexpr float GYRO_LSB_PER_DPS = 16.4f;    // MPU6050 +/-2000 deg/s
constexpr int16_t CLIP_RAW = 32600;

constexpr uint32_t SAMPLE_INTERVAL_US = 1000; // 1 kHz target
constexpr uint32_t SERIAL_BAUD = 115200;

// Set this to 1 while collecting traces. Serial output becomes:
// ms,ax_g,ay_g,az_g,accel_mag_g,dynamic_g,gx_dps,gy_dps,gz_dps,gyro_mag_dps
constexpr bool RAW_SERIAL_LOGGING = false;

// BLE UUIDs. Keep these fixed so the mobile app can discover the device.
constexpr char SERVICE_UUID[] = "9b20f001-5f3a-4b8d-9f1c-7fd95a6f0001";
constexpr char SHOT_EVENT_UUID[] = "9b20f002-5f3a-4b8d-9f1c-7fd95a6f0001";
constexpr char TELEMETRY_UUID[] = "9b20f003-5f3a-4b8d-9f1c-7fd95a6f0001";
constexpr char CONFIG_UUID[] = "9b20f004-5f3a-4b8d-9f1c-7fd95a6f0001";
constexpr char FIRMWARE_UUID[] = "9b20f005-5f3a-4b8d-9f1c-7fd95a6f0001";

constexpr uint8_t PACKET_MAGIC = 0x46; // 'F'
constexpr uint8_t PACKET_TYPE_SHOT = 0x01;
constexpr uint8_t PACKET_TYPE_TELEMETRY = 0x02;

enum ContactResult : uint8_t {
  RESULT_MISS = 0,
  RESULT_WEAK = 1,
  RESULT_GOOD = 2,
  RESULT_SOLID = 3,
};

enum DetectorState : uint8_t {
  STATE_IDLE = 0,
  STATE_SHOT_WINDOW = 1,
  STATE_IMPACT_SETTLE = 2,
  STATE_COOLDOWN = 3,
};

struct DetectorConfig {
  float swingStartDps = 380.0f;
  float impactDynamicG = 2.6f;
  float impactJerkG = 1.2f;
  float goodContactG = 5.5f;
  float solidContactG = 10.5f;
  uint16_t impactWindowMs = 280;
  uint16_t impactSettleMs = 12;
  uint16_t cooldownMs = 650;
};

struct RawSample {
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t temp = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
};

struct ShotContext {
  uint32_t swingStartMs = 0;
  uint32_t impactFirstMs = 0;
  float peakGyroDps = 0.0f;
  float peakDynamicG = 0.0f;
  float peakJerkG = 0.0f;
  bool impactSeen = false;
  bool clipped = false;
};

struct __attribute__((packed)) ShotEventPacket {
  uint8_t magic;
  uint8_t type;
  uint16_t sequence;
  uint8_t result;
  uint8_t flags;          // bit0: hit, bit1: accel clipped
  uint16_t peakAccelCg;   // centi-g, dynamic acceleration above gravity
  uint16_t peakGyroDps;
  uint16_t impactDelayMs; // 0xFFFF for misses
  uint8_t batteryPercent;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t magic;
  uint8_t type;
  uint16_t batteryMv;
  uint8_t batteryPercent;
  uint16_t sampleRateHz;
  uint8_t detectorState;
  uint32_t uptimeSec;
};

static_assert(sizeof(ShotEventPacket) <= 20, "Shot packet must fit default BLE MTU");
static_assert(sizeof(TelemetryPacket) <= 20, "Telemetry packet must fit default BLE MTU");

uint8_t mpuAddress = MPU_ADDR_LOW;
DetectorConfig config;
DetectorState detectorState = STATE_IDLE;
ShotContext shot;

NimBLECharacteristic* shotCharacteristic = nullptr;
NimBLECharacteristic* telemetryCharacteristic = nullptr;
NimBLECharacteristic* configCharacteristic = nullptr;
NimBLECharacteristic* batteryCharacteristic = nullptr;

bool bleConnected = false;
uint16_t shotSequence = 0;
uint32_t lastEventMs = 0;
uint32_t nextSampleUs = 0;
uint32_t lastBatteryUpdateMs = 0;
uint32_t sampleCounter = 0;
uint32_t lastSampleRateMs = 0;
uint16_t measuredSampleRateHz = 0;
uint16_t batteryMv = 0;
uint8_t batteryPercent = 0;
float previousAccelMagG = 1.0f;

uint16_t clampU16(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 65535.0f) return 65535;
  return static_cast<uint16_t>(value + 0.5f);
}

uint8_t lithiumPercentFromMv(uint16_t mv) {
  struct Point {
    uint16_t mv;
    uint8_t pct;
  };

  constexpr Point curve[] = {
    {4200, 100}, {4100, 90}, {4000, 80}, {3900, 65}, {3800, 50},
    {3700, 35},  {3600, 20}, {3500, 10}, {3400, 5},  {3300, 0},
  };

  if (mv >= curve[0].mv) return 100;
  for (size_t i = 1; i < sizeof(curve) / sizeof(curve[0]); ++i) {
    if (mv >= curve[i].mv) {
      const Point high = curve[i - 1];
      const Point low = curve[i];
      const float span = static_cast<float>(high.mv - low.mv);
      const float pos = static_cast<float>(mv - low.mv) / span;
      return static_cast<uint8_t>(low.pct + pos * (high.pct - low.pct) + 0.5f);
    }
  }
  return 0;
}

uint16_t readBatteryMilliVolts() {
  uint32_t totalMv = 0;
  constexpr uint8_t reads = 12;

  for (uint8_t i = 0; i < reads; ++i) {
    totalMv += analogReadMilliVolts(BATTERY_ADC_PIN);
    delayMicroseconds(250);
  }

  const float adcMv = static_cast<float>(totalMv) / reads;
  const float divider = (BATTERY_R1_OHMS + BATTERY_R2_OHMS) / BATTERY_R2_OHMS;
  const float packMv = adcMv * divider * BATTERY_CALIBRATION;
  return clampU16(packMv);
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMpuRegister(uint8_t address, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(address), 1, true) != 1) return false;
  value = Wire.read();
  return true;
}

bool detectMpuAddress() {
  uint8_t who = 0;
  if (readMpuRegister(MPU_ADDR_LOW, REG_WHO_AM_I, who) && who == 0x68) {
    mpuAddress = MPU_ADDR_LOW;
    return true;
  }
  if (readMpuRegister(MPU_ADDR_HIGH, REG_WHO_AM_I, who) && who == 0x68) {
    mpuAddress = MPU_ADDR_HIGH;
    return true;
  }
  return false;
}

bool setupMpu6050() {
  if (!detectMpuAddress()) return false;

  // Wake up, use X gyro PLL, no sample divider, widest practical DLPF,
  // accelerometer +/-16 g, gyroscope +/-2000 dps.
  bool ok = true;
  ok &= writeMpuRegister(REG_PWR_MGMT_1, 0x01);
  delay(50);
  ok &= writeMpuRegister(REG_SMPLRT_DIV, 0x00);
  ok &= writeMpuRegister(REG_CONFIG, 0x00);
  ok &= writeMpuRegister(REG_ACCEL_CONFIG, 0x18);
  ok &= writeMpuRegister(REG_GYRO_CONFIG, 0x18);
  return ok;
}

bool readMpuRaw(RawSample& sample) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(mpuAddress), 14, true) != 14) return false;

  auto read16 = []() -> int16_t {
    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    return static_cast<int16_t>((high << 8) | low);
  };

  sample.ax = read16();
  sample.ay = read16();
  sample.az = read16();
  sample.temp = read16();
  sample.gx = read16();
  sample.gy = read16();
  sample.gz = read16();
  return true;
}

bool axisClipped(const RawSample& sample) {
  return abs(sample.ax) >= CLIP_RAW || abs(sample.ay) >= CLIP_RAW ||
         abs(sample.az) >= CLIP_RAW;
}

ContactResult classifyContact(float peakDynamicG, bool clipped) {
  if (clipped || peakDynamicG >= config.solidContactG) return RESULT_SOLID;
  if (peakDynamicG >= config.goodContactG) return RESULT_GOOD;
  return RESULT_WEAK;
}

String configText() {
  String text;
  text.reserve(128);
  text += "SWING=" + String(config.swingStartDps, 1);
  text += ";IMPACT=" + String(config.impactDynamicG, 2);
  text += ";JERK=" + String(config.impactJerkG, 2);
  text += ";GOOD=" + String(config.goodContactG, 2);
  text += ";SOLID=" + String(config.solidContactG, 2);
  text += ";WINDOW=" + String(config.impactWindowMs);
  text += ";SETTLE=" + String(config.impactSettleMs);
  text += ";COOLDOWN=" + String(config.cooldownMs);
  return text;
}

void updateConfigCharacteristic() {
  if (configCharacteristic == nullptr) return;
  const String text = configText();
  configCharacteristic->setValue(reinterpret_cast<const uint8_t*>(text.c_str()), text.length());
}

void applyConfigCommand(String command) {
  command.trim();
  command.toUpperCase();
  command.replace(",", ";");

  int start = 0;
  while (start < command.length()) {
    int end = command.indexOf(';', start);
    if (end < 0) end = command.length();

    String token = command.substring(start, end);
    token.trim();
    const int equals = token.indexOf('=');
    if (equals > 0) {
      const String key = token.substring(0, equals);
      const float value = token.substring(equals + 1).toFloat();

      if (key == "SWING" && value >= 100.0f && value <= 2500.0f) config.swingStartDps = value;
      if (key == "IMPACT" && value >= 0.5f && value <= 16.0f) config.impactDynamicG = value;
      if (key == "JERK" && value >= 0.1f && value <= 16.0f) config.impactJerkG = value;
      if (key == "GOOD" && value >= 1.0f && value <= 16.0f) config.goodContactG = value;
      if (key == "SOLID" && value >= 1.0f && value <= 16.0f) config.solidContactG = value;
      if (key == "WINDOW" && value >= 80.0f && value <= 800.0f) config.impactWindowMs = value;
      if (key == "SETTLE" && value >= 4.0f && value <= 40.0f) config.impactSettleMs = value;
      if (key == "COOLDOWN" && value >= 200.0f && value <= 2000.0f) config.cooldownMs = value;
    }

    start = end + 1;
  }

  updateConfigCharacteristic();
  Serial.print("CONFIG ");
  Serial.println(configText());
}

void sendTelemetry(bool notify) {
  if (telemetryCharacteristic == nullptr) return;

  TelemetryPacket packet{};
  packet.magic = PACKET_MAGIC;
  packet.type = PACKET_TYPE_TELEMETRY;
  packet.batteryMv = batteryMv;
  packet.batteryPercent = batteryPercent;
  packet.sampleRateHz = measuredSampleRateHz;
  packet.detectorState = detectorState;
  packet.uptimeSec = millis() / 1000UL;

  telemetryCharacteristic->setValue(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  if (notify && bleConnected) telemetryCharacteristic->notify();

  if (batteryCharacteristic != nullptr) {
    batteryCharacteristic->setValue(&batteryPercent, 1);
    if (notify && bleConnected) batteryCharacteristic->notify();
  }
}

void updateBatteryIfDue(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - lastBatteryUpdateMs < BATTERY_UPDATE_MS) return;
  lastBatteryUpdateMs = now;

  batteryMv = readBatteryMilliVolts();
  batteryPercent = lithiumPercentFromMv(batteryMv);
  sendTelemetry(true);

  if (batteryMv > 0 && batteryMv <= 3400) {
    Serial.print("BATTERY_CRITICAL ");
    Serial.print(batteryMv);
    Serial.println("mV");
  } else if (batteryMv > 0 && batteryMv <= 3500) {
    Serial.print("BATTERY_LOW ");
    Serial.print(batteryMv);
    Serial.println("mV");
  }
}

void sendShotEvent(ContactResult result) {
  const bool hit = result != RESULT_MISS;
  ShotEventPacket packet{};
  packet.magic = PACKET_MAGIC;
  packet.type = PACKET_TYPE_SHOT;
  packet.sequence = ++shotSequence;
  packet.result = result;
  packet.flags = (hit ? 0x01 : 0x00) | (shot.clipped ? 0x02 : 0x00);
  packet.peakAccelCg = clampU16(shot.peakDynamicG * 100.0f);
  packet.peakGyroDps = clampU16(shot.peakGyroDps);
  packet.impactDelayMs = hit ? clampU16(static_cast<float>(shot.impactFirstMs - shot.swingStartMs)) : 0xFFFF;
  packet.batteryPercent = batteryPercent;

  if (shotCharacteristic != nullptr) {
    shotCharacteristic->setValue(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    if (bleConnected) shotCharacteristic->notify();
  }

  Serial.print("SHOT seq=");
  Serial.print(packet.sequence);
  Serial.print(" result=");
  Serial.print(result == RESULT_MISS ? "MISS" : result == RESULT_WEAK ? "WEAK" : result == RESULT_GOOD ? "GOOD" : "SOLID");
  Serial.print(" peak_g=");
  Serial.print(shot.peakDynamicG, 2);
  Serial.print(" gyro_dps=");
  Serial.print(shot.peakGyroDps, 0);
  Serial.print(" impact_ms=");
  if (hit) Serial.print(packet.impactDelayMs);
  else Serial.print("none");
  Serial.print(" battery=");
  Serial.print(batteryPercent);
  Serial.println("%");

  lastEventMs = millis();
  detectorState = STATE_COOLDOWN;
}

void resetShot(uint32_t nowMs, float gyroDps, float dynamicG, float jerkG, bool clipped) {
  shot = ShotContext{};
  shot.swingStartMs = nowMs;
  shot.peakGyroDps = gyroDps;
  shot.peakDynamicG = dynamicG;
  shot.peakJerkG = jerkG;
  shot.clipped = clipped;
  detectorState = STATE_SHOT_WINDOW;
}

void updateShotPeaks(float gyroDps, float dynamicG, float jerkG, bool clipped) {
  if (gyroDps > shot.peakGyroDps) shot.peakGyroDps = gyroDps;
  if (dynamicG > shot.peakDynamicG) shot.peakDynamicG = dynamicG;
  if (jerkG > shot.peakJerkG) shot.peakJerkG = jerkG;
  shot.clipped = shot.clipped || clipped;
}

bool isImpactSample(float dynamicG, float jerkG, bool clipped) {
  if (clipped) return true;
  return dynamicG >= config.impactDynamicG && jerkG >= config.impactJerkG;
}

void processMotionSample() {
  RawSample raw;
  if (!readMpuRaw(raw)) return;

  ++sampleCounter;

  const float axG = raw.ax / ACCEL_LSB_PER_G;
  const float ayG = raw.ay / ACCEL_LSB_PER_G;
  const float azG = raw.az / ACCEL_LSB_PER_G;
  const float gxDps = raw.gx / GYRO_LSB_PER_DPS;
  const float gyDps = raw.gy / GYRO_LSB_PER_DPS;
  const float gzDps = raw.gz / GYRO_LSB_PER_DPS;

  const float accelMagG = sqrtf(axG * axG + ayG * ayG + azG * azG);
  const float dynamicG = fabsf(accelMagG - 1.0f);
  const float jerkG = fabsf(accelMagG - previousAccelMagG);
  previousAccelMagG = accelMagG;

  const float gyroMagDps = sqrtf(gxDps * gxDps + gyDps * gyDps + gzDps * gzDps);
  const bool clipped = axisClipped(raw);
  const uint32_t nowMs = millis();

  if (RAW_SERIAL_LOGGING) {
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(axG, 3);
    Serial.print(',');
    Serial.print(ayG, 3);
    Serial.print(',');
    Serial.print(azG, 3);
    Serial.print(',');
    Serial.print(accelMagG, 3);
    Serial.print(',');
    Serial.print(dynamicG, 3);
    Serial.print(',');
    Serial.print(gxDps, 1);
    Serial.print(',');
    Serial.print(gyDps, 1);
    Serial.print(',');
    Serial.print(gzDps, 1);
    Serial.print(',');
    Serial.println(gyroMagDps, 1);
  }

  switch (detectorState) {
    case STATE_IDLE:
      if (nowMs - lastEventMs >= config.cooldownMs && gyroMagDps >= config.swingStartDps) {
        resetShot(nowMs, gyroMagDps, dynamicG, jerkG, clipped);
        if (isImpactSample(dynamicG, jerkG, clipped)) {
          shot.impactSeen = true;
          shot.impactFirstMs = nowMs;
          detectorState = STATE_IMPACT_SETTLE;
        }
      }
      break;

    case STATE_SHOT_WINDOW:
      updateShotPeaks(gyroMagDps, dynamicG, jerkG, clipped);
      if (isImpactSample(dynamicG, jerkG, clipped)) {
        shot.impactSeen = true;
        shot.impactFirstMs = nowMs;
        detectorState = STATE_IMPACT_SETTLE;
      } else if (nowMs - shot.swingStartMs >= config.impactWindowMs) {
        sendShotEvent(RESULT_MISS);
      }
      break;

    case STATE_IMPACT_SETTLE:
      updateShotPeaks(gyroMagDps, dynamicG, jerkG, clipped);
      if (nowMs - shot.impactFirstMs >= config.impactSettleMs) {
        sendShotEvent(classifyContact(shot.peakDynamicG, shot.clipped));
      }
      break;

    case STATE_COOLDOWN:
      if (nowMs - lastEventMs >= config.cooldownMs) {
        detectorState = STATE_IDLE;
      }
      break;
  }
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    bleConnected = true;
    server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 60);
    updateBatteryIfDue(true);
    Serial.println("BLE connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)server;
    (void)connInfo;
    (void)reason;
    bleConnected = false;
    NimBLEDevice::getAdvertising()->start();
    Serial.println("BLE disconnected, advertising restarted");
  }
};

class ConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    const std::string value = characteristic->getValue();
    if (!value.empty()) {
      applyConfigCommand(String(value.c_str()));
    }
  }
};

void setupBle() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0);
  NimBLEDevice::setMTU(64);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);
  shotCharacteristic = service->createCharacteristic(
      SHOT_EVENT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  telemetryCharacteristic = service->createCharacteristic(
      TELEMETRY_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  configCharacteristic = service->createCharacteristic(
      CONFIG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLECharacteristic* firmwareCharacteristic = service->createCharacteristic(
      FIRMWARE_UUID, NIMBLE_PROPERTY::READ);

  configCharacteristic->setCallbacks(new ConfigCallbacks());
  firmwareCharacteristic->setValue(FIRMWARE_VERSION);
  updateConfigCharacteristic();
  service->start();

  NimBLEService* batteryService = server->createService(NimBLEUUID(static_cast<uint16_t>(0x180F)));
  batteryCharacteristic = batteryService->createCharacteristic(
      NimBLEUUID(static_cast<uint16_t>(0x2A19)),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  batteryService->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->addServiceUUID(NimBLEUUID(static_cast<uint16_t>(0x180F)));
  advertising->setName(DEVICE_NAME);
  advertising->setAppearance(0x0540); // Generic sensor
  advertising->setMinInterval(160);   // 100 ms units of 0.625 ms
  advertising->setMaxInterval(320);   // 200 ms
  advertising->start();

  Serial.println("BLE advertising as FieldSense-Bat");
}

void updateSampleRateIfDue() {
  const uint32_t now = millis();
  if (now - lastSampleRateMs < 1000) return;

  measuredSampleRateHz = sampleCounter;
  sampleCounter = 0;
  lastSampleRateMs = now;
}

} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  setCpuFrequencyMhz(80);
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println();
  Serial.println("FieldSense ESP32 starting");

  if (!setupMpu6050()) {
    Serial.println("ERROR MPU6050 not found. Check VCC, GND, SDA=21, SCL=22.");
    while (true) {
      delay(1000);
    }
  }

  updateBatteryIfDue(true);
  setupBle();
  sendTelemetry(false);

  nextSampleUs = micros();
  lastSampleRateMs = millis();
  Serial.print("MPU6050 ready at I2C 0x");
  Serial.println(mpuAddress, HEX);
}

void loop() {
  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_INTERVAL_US;
    if (static_cast<int32_t>(micros() - nextSampleUs) > static_cast<int32_t>(SAMPLE_INTERVAL_US)) {
      nextSampleUs = micros() + SAMPLE_INTERVAL_US;
    }
    processMotionSample();
  }

  updateSampleRateIfDue();
  updateBatteryIfDue(false);
}
