/*
 * FieldSense — ESP32 + MPU6050 bat sensor
 * Paired with app.html via Web Bluetooth (Chrome on Android).
 *
 * BLE layout — UUIDs MUST match app.html BLE constants exactly:
 *   SVC  8b9e0000-1f4a-4b8b-9c1b-6e2a3d4f5a01
 *   EVT  8b9e0001  NOTIFY  — JSON shot events
 *   CFG  8b9e0002  RD/WR   — 1-byte commands or "KEY=VAL;..." config string
 *   STA  8b9e0003  NOTIFY  — JSON telemetry ~2 Hz
 *
 * JSON shot event (what app.html _handle expects):
 *   {"swing":1,"hit":1,"a":5.2,"g":420,"id":3}
 *   "a" = dynamic g (impact strength), "g" = peak gyro deg/s
 *
 * CFG 1-byte commands (from app.html setSport / calibrate):
 *   0x00 = cricket, 0x01 = paddleball, 0x02 = pickleball
 *   0x5A = calibrate (zero gyro/accel bias, hold bat still)
 *
 * Requirements:
 *   ESP32 Arduino core 3.0.x  (NOT 3.3.x — linker bug breaks BLE)
 *   Board: "ESP32 Dev Module"  No extra libraries needed.
 *
 * This device does NOT appear in Android Settings → Bluetooth.
 * Open app.html in Chrome on Android, tap CONNECT SENSOR.
 */

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ── MPU6050 registers ─────────────────────────────────────────────────
#define MPU_ADDR      0x68
#define WHO_AM_I      0x75
#define PWR_MGMT_1    0x6B
#define SMPLRT_DIV    0x19
#define CONFIG_REG    0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B

const float ACCEL_SCALE = 16.0f   / 32768.0f;   // ±16 g range
const float GYRO_SCALE  = 2000.0f / 32768.0f;   // ±2000 °/s range

// ── MAX9814 microphone (acoustic impact confirmation) ─────────────────
// MAX9814 OUT → an ESP32 ADC1 pin (GPIO32-39). GPIO34 is input-only and
// ADC1 stays usable while BLE radio is active (ADC2 does NOT — don't use it).
// Wiring: VDD→3V3, GND→GND, OUT→GPIO34, Gain→floating(60dB) or GND(50dB).
#define MIC_PIN        34
float  micBias     = 0.0f;    // running DC baseline (ADC counts)
float  peakSound   = 0.0f;    // loudest deviation during current shot window
uint16_t soundLevel = 0;      // rolling max deviation, for live tuning telemetry

// ── BLE UUIDs — must match app.html BLE object exactly ───────────────
#define SVC_UUID  "8b9e0000-1f4a-4b8b-9c1b-6e2a3d4f5a01"
#define EVT_UUID  "8b9e0001-1f4a-4b8b-9c1b-6e2a3d4f5a01"
#define CFG_UUID  "8b9e0002-1f4a-4b8b-9c1b-6e2a3d4f5a01"
#define STA_UUID  "8b9e0003-1f4a-4b8b-9c1b-6e2a3d4f5a01"

BLECharacteristic *evtChar, *cfgChar, *staChar;
bool deviceConnected = false;

// ── Sport (0=cricket, 1=paddleball, 2=pickleball) ─────────────────────
uint8_t currentSport = 0;

// ── Tunable detection thresholds ─────────────────────────────────────
struct Config {
  float    swingGyro   = 380.0f;  // °/s — opens shot window
  float    impactG     = 2.6f;    // min peak dynamic g to consider a contact
  float    impactJerk  = 1.5f;    // min g JUMP between samples — THE hit/swing test
  float    goodG       = 5.5f;    // GOOD hit threshold
  float    solidG      = 10.5f;   // SOLID hit threshold
  uint16_t windowMs    = 280;     // impact-listen window (ms)
  uint16_t cooldownMs  = 650;     // lockout after each shot
  uint16_t soundThresh = 350;     // mic deviation (ADC counts) — used only if useMic
  uint8_t  useMic      = 0;       // 0 = mic is informational only; 1 = also require sound
} cfg;

String buildCfgString() {
  char b[128];
  snprintf(b, sizeof(b),
    "SWING=%.0f;IMPACT=%.1f;JERK=%.1f;GOOD=%.1f;SOLID=%.1f;WINDOW=%u;COOLDOWN=%u;SOUND=%u;MIC=%u;SPORT=%u",
    cfg.swingGyro, cfg.impactG, cfg.impactJerk, cfg.goodG, cfg.solidG,
    cfg.windowMs, cfg.cooldownMs, cfg.soundThresh, cfg.useMic, currentSport);
  return String(b);
}

void parseConfigString(const String& s) {
  int start = 0;
  while (start < (int)s.length()) {
    int eq   = s.indexOf('=', start); if (eq < 0) break;
    int semi = s.indexOf(';', eq);    if (semi < 0) semi = s.length();
    String key = s.substring(start, eq);
    float  val = s.substring(eq + 1, semi).toFloat();
    if      (key == "SWING")    cfg.swingGyro  = val;
    else if (key == "IMPACT")   cfg.impactG    = val;
    else if (key == "JERK")     cfg.impactJerk = val;
    else if (key == "GOOD")     cfg.goodG      = val;
    else if (key == "SOLID")    cfg.solidG     = val;
    else if (key == "WINDOW")   cfg.windowMs   = (uint16_t)val;
    else if (key == "COOLDOWN") cfg.cooldownMs = (uint16_t)val;
    else if (key == "SOUND")    cfg.soundThresh= (uint16_t)val;
    else if (key == "MIC")      cfg.useMic     = (uint8_t)val;
    else if (key == "SPORT")    currentSport   = (uint8_t)val;
    start = semi + 1;
  }
  Serial.print("Config: "); Serial.println(buildCfgString());
}

// ── Calibration offsets ───────────────────────────────────────────────
float axOff=0, ayOff=0, azOff=0;
float gxOff=0, gyOff=0, gzOff=0;
volatile bool calibRequested = false;

// ── Battery (stub — replace with ADC divider when LiPo is wired) ─────
uint16_t readBattMv()  { return 4000; }
uint8_t  battPct(uint16_t mv) {
  if (mv >= 4200) return 100;
  if (mv <= 3300) return 0;
  return (uint8_t)((mv - 3300) * 100 / 900);
}

// ── BLE callbacks ─────────────────────────────────────────────────────
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    Serial.println(">> app connected");
    cfgChar->setValue(buildCfgString().c_str());
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    Serial.println(">> app disconnected — re-advertising");
    delay(200);
    BLEDevice::startAdvertising();
  }
};

// App writes 1 byte for sport/calibrate, or a "KEY=VAL;..." config string
class CfgCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.length() == 1) {
      uint8_t b = (uint8_t)v[0];
      if (b == 0x5A) {
        calibRequested = true;
        Serial.println("Calibrate requested — hold bat still");
      } else if (b <= 2) {
        currentSport = b;
        static const char* names[] = { "cricket", "paddleball", "pickleball" };
        Serial.printf("Sport: %s\n", names[b]);
      }
    } else if (v.length() > 1) {
      parseConfigString(v);
    }
    cfgChar->setValue(buildCfgString().c_str());
  }
};

// ── MPU6050 ───────────────────────────────────────────────────────────
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
uint8_t mpuRead1(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
bool setupMPU() {
  uint8_t who = mpuRead1(WHO_AM_I);
  Serial.printf("MPU6050 WHO_AM_I=0x%02X\n", who);
  if (who == 0xFF) return false;
  mpuWrite(PWR_MGMT_1,   0x00); delay(50);  // wake up
  mpuWrite(SMPLRT_DIV,   0x00);             // max sample rate
  mpuWrite(CONFIG_REG,   0x00);             // no DLPF
  mpuWrite(GYRO_CONFIG,  0x18);             // ±2000 °/s
  mpuWrite(ACCEL_CONFIG, 0x18);             // ±16 g
  return true;
}

void readIMU(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(ACCEL_XOUT_H); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire.available() < 14) { ax=ay=0; az=1; gx=gy=gz=0; return; }
  int16_t rax=(Wire.read()<<8)|Wire.read(), ray=(Wire.read()<<8)|Wire.read(), raz=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t rgx=(Wire.read()<<8)|Wire.read(), rgy=(Wire.read()<<8)|Wire.read(), rgz=(Wire.read()<<8)|Wire.read();
  ax = rax*ACCEL_SCALE - axOff;
  ay = ray*ACCEL_SCALE - ayOff;
  az = raz*ACCEL_SCALE - azOff;
  gx = rgx*GYRO_SCALE  - gxOff;
  gy = rgy*GYRO_SCALE  - gyOff;
  gz = rgz*GYRO_SCALE  - gzOff;
}

// ── MAX9814 microphone ────────────────────────────────────────────────
// Returns |sample - DC bias| in ADC counts. A bat/ball contact is a sharp
// acoustic transient that an empty swing simply does not produce. Always
// reads the pin so `snd` reflects reality for tuning, even before MIC=1.
float micDeviation() {
  return fabsf((float)analogRead(MIC_PIN) - micBias);
}

// Configure the ADC and learn the quiet DC bias. The boot print tells you if
// the module is wired: a healthy MAX9814 idles around mid-rail (~1400-1700 at
// 11dB). A bias pinned near 0 or 4095, or a huge span, means OUT isn't
// connected to MIC_PIN (or wrong pin / using an ADC2 pin).
void setupMic() {
  analogReadResolution(12);                      // 0..4095
  analogSetPinAttenuation(MIC_PIN, ADC_11db);    // full ~0..3.1 V range
  long sum = 0; int mn = 4095, mx = 0;
  const int N = 256;
  for (int i = 0; i < N; i++) {
    int s = analogRead(MIC_PIN);
    sum += s; if (s < mn) mn = s; if (s > mx) mx = s;
    delayMicroseconds(150);
  }
  micBias = sum / (float)N;
  Serial.printf("MIC on GPIO%d: bias=%.0f span=%d  "
                "(bias ~1500 = wired OK; near 0/4095 or huge span = check wiring)\n",
                MIC_PIN, micBias, mx - mn);
}

// Gather ~100 still samples and store bias — call only when bat is stationary
void doCalibrate() {
  float sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  axOff=ayOff=azOff=gxOff=gyOff=gzOff=0;
  for (int i = 0; i < 100; i++) {
    float ax,ay,az,gx,gy,gz; readIMU(ax,ay,az,gx,gy,gz);
    sax+=ax; say+=ay; saz+=az; sgx+=gx; sgy+=gy; sgz+=gz;
    delay(5);
  }
  axOff=sax/100; ayOff=say/100; azOff=(saz/100)-1.0f;  // keep 1 g on Z
  gxOff=sgx/100; gyOff=sgy/100; gzOff=sgz/100;
  Serial.printf("Calib done: ax=%.3f ay=%.3f az=%.3f gx=%.1f gy=%.1f gz=%.1f\n",
                axOff,ayOff,azOff,gxOff,gyOff,gzOff);
}

// ── Shot detection state machine ──────────────────────────────────────
enum DetState { IDLE, SHOT_WINDOW, COOLDOWN };
DetState detState = IDLE;
uint16_t shotSeq = 0;
unsigned long windowStart=0, cooldownStart=0;
float peakGyro=0, peakAccel=0, peakJump=0;
float prevAccelDyn=0;   // last loop's dynamic g — for sample-to-sample jerk

// Send JSON event — matches what app.html _handle() parses
//   {"swing":1,"hit":1,"a":5.2,"g":420,"id":3}
void sendShot(bool hit, float g, float gyro) {
  if (!deviceConnected) return;
  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"swing\":1,\"hit\":%d,\"a\":%.1f,\"g\":%.0f,\"id\":%u}",
    hit ? 1 : 0, g, gyro, (unsigned)shotSeq++);
  Serial.printf("SHOT hit=%d a=%.1f(>%.1f) jrk=%.1f(>%.1f) g=%.0f snd=%.0f(>%u) mic=%u id=%u\n",
                hit, g, cfg.impactG, peakJump, cfg.impactJerk, gyro,
                peakSound, cfg.soundThresh, cfg.useMic, shotSeq-1);
  evtChar->setValue((uint8_t*)buf, strlen(buf));
  evtChar->notify();
}

// ── Telemetry ─────────────────────────────────────────────────────────
unsigned long lastTelMs=0, sampleWinStart=0;
uint32_t sampleCount=0;
uint16_t measuredHz=0;

void sendTelemetry() {
  if (!deviceConnected) return;
  uint8_t bp = battPct(readBattMv());
  char buf[112];
  snprintf(buf, sizeof(buf),
    "{\"batt\":%u,\"hz\":%u,\"up\":%lu,\"state\":%u,\"mic\":%u,\"bias\":%.0f,\"snd\":%u}",
    bp, measuredHz, millis()/1000, (unsigned)detState,
    cfg.useMic, micBias, soundLevel);
  staChar->setValue((uint8_t*)buf, strlen(buf));
  staChar->notify();
  soundLevel = 0;   // reset rolling peak each telemetry frame
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("\nVocalWillow booting...");

  Wire.begin(21, 22);         // SDA=GPIO21, SCL=GPIO22 (ESP32 default)
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  if (!setupMPU()) Serial.println("!! MPU6050 not found — check wiring. BLE starts anyway.");
  else             Serial.println("MPU6050 OK.");

  setupMic();   // configure ADC + learn mic DC bias (prints it for diagnosis)

  BLEDevice::init("VocalWillow");
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());

  BLEService* svc = srv->createService(SVC_UUID);

  evtChar = svc->createCharacteristic(EVT_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  evtChar->addDescriptor(new BLE2902());

  cfgChar = svc->createCharacteristic(CFG_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  cfgChar->setCallbacks(new CfgCB());
  cfgChar->setValue(buildCfgString().c_str());

  staChar = svc->createCharacteristic(STA_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  staChar->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Advertising as 'FieldSense'.");
  Serial.println("Open app.html in Chrome (Android) → tap CONNECT SENSOR.");
  sampleWinStart = millis();
}

// ── Main loop ─────────────────────────────────────────────────────────
void loop() {
  // Handle calibrate request between shots (keeps IMU read timing stable)
  if (calibRequested && detState == IDLE) {
    doCalibrate();
    calibRequested = false;
  }

  float ax,ay,az,gx,gy,gz;
  readIMU(ax,ay,az,gx,gy,gz);

  float gyroMag  = sqrtf(gx*gx + gy*gy + gz*gz);
  float accelMag = sqrtf(ax*ax + ay*ay + az*az);
  float accelDyn = fabsf(accelMag - 1.0f);    // subtract 1g gravity baseline

  // Sample the mic every loop so we never miss the contact transient,
  // and keep a rolling level for live tuning in telemetry.
  float micDev = micDeviation();
  if (micDev > soundLevel) soundLevel = (uint16_t)micDev;

  unsigned long now = millis();

  // Track real sample rate
  sampleCount++;
  if (now - sampleWinStart >= 1000) {
    measuredHz = sampleCount; sampleCount = 0; sampleWinStart = now;
  }

  // Sudden sample-to-sample rise in dynamic g. A ball impact spikes hard in
  // one sample (large jump); an empty swing ramps up smoothly (tiny jumps).
  float accelJump = accelDyn - prevAccelDyn;

  switch (detState) {
    case IDLE:
      // Slowly track the mic's resting DC level so bias drift doesn't
      // masquerade as sound (only while idle, never mid-swing).
      micBias += (analogRead(MIC_PIN) - micBias) * 0.002f;
      if (gyroMag > cfg.swingGyro) {
        detState  = SHOT_WINDOW;
        peakGyro  = gyroMag;
        peakAccel = accelDyn;   // seed peaks at swing onset and track from here
        peakJump  = 0;
        peakSound = 0;
        windowStart = now;
      }
      break;

    case SHOT_WINDOW:
      // Track everything across the whole window so the contact spike is caught
      // wherever it lands relative to the gyro peak.
      if (gyroMag   > peakGyro)  peakGyro  = gyroMag;
      if (accelDyn  > peakAccel) peakAccel = accelDyn;
      if (accelJump > peakJump)  peakJump  = accelJump;
      if (micDev    > peakSound) peakSound = micDev;
      if (now - windowStart > cfg.windowMs) {
        bool energyHit = peakAccel >= cfg.impactG;     // enough magnitude
        bool sharpHit  = peakJump  >= cfg.impactJerk;  // sudden = real contact
        bool soundHit  = peakSound >= cfg.soundThresh; // acoustic crack
        // A hit must be a sharp spike WITH enough energy — this is what rejects
        // empty swings. Sound is OFF by default (informational only); set MIC=1
        // to additionally require the acoustic crack once you've tuned SOUND.
        bool hit = energyHit && sharpHit;
        if (cfg.useMic) hit = hit && soundHit;
        sendShot(hit, peakAccel, peakGyro);
        detState = COOLDOWN;
        cooldownStart = now;
      }
      break;

    case COOLDOWN:
      if (now - cooldownStart > cfg.cooldownMs) detState = IDLE;
      break;
  }

  prevAccelDyn = accelDyn;   // for next loop's jerk calc

  // Push telemetry every 2 s
  if (now - lastTelMs > 2000) { sendTelemetry(); lastTelMs = now; }

  delayMicroseconds(1500);   // ~600 Hz sample rate
}
