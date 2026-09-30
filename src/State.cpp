#include "State.h"
#include <Preferences.h>

RideState g_ride;
Settings  g_cfg;

static Preferences prefs;

namespace {

// Settings are saved from two tasks: the link task when a device is picked or
// forgotten, and the main loop for gear and cadence-source changes. They share
// one Preferences object, so a concurrent begin() would leave one task holding
// a handle the other has already closed.
SemaphoreHandle_t s_prefsMutex = nullptr;

// Created on the first load(), which happens in setup() before any other task
// exists - so the lazy initialisation itself is never contended.
SemaphoreHandle_t prefsMutex() {
  if (s_prefsMutex == nullptr) s_prefsMutex = xSemaphoreCreateMutex();
  return s_prefsMutex;
}

// Address bytes and type travel together as one 7-byte blob.
void loadDevice(Preferences& store, const char* key, SavedDevice& out) {
  uint8_t blob[7] = {0};
  // isKey first: reading or erasing a missing key logs an ESP error, and three
  // of those every boot bury anything worth reading.
  if (store.isKey(key) && store.getBytes(key, blob, sizeof(blob)) == sizeof(blob)) {
    memcpy(out.addr, blob, 6);
    out.type = blob[6];
    out.valid = true;
  } else {
    out.valid = false;
  }
}

float loadFloat(Preferences& store, const char* key, float fallback) {
  return store.isKey(key) ? store.getFloat(key, fallback) : fallback;
}

// Step and range of each profile field, in ProfileField order.
struct ProfileSpec {
  float step;
  float min;
  float max;
};
constexpr ProfileSpec kProfileSpecs[kProfileFieldCount] = {
    {0.5f, 30.0f, 200.0f},     // rider, kg
    {0.1f, 3.0f, 30.0f},       // bike, kg
    {1.0f, 50.0f, 600.0f},     // FTP, W
    {1.0f, 1500.0f, 2500.0f},  // wheel circumference, mm
    {1.0f, 120.0f, 220.0f},    // rider height, cm
};

float& profileSetting(Settings& settings, ProfileField field) {
  switch (field) {
    case ProfileField::RiderKg: return settings.riderKg;
    case ProfileField::BikeKg: return settings.bikeKg;
    case ProfileField::FtpW: return settings.ftpW;
    case ProfileField::WheelMm: return settings.wheelMm;
    default: return settings.riderCm;
  }
}

// Power is sampled four times a second into a 3 s window, and the window's
// average is recorded once a second. The value held between two trainer
// notifications counts for as long as it was held, which is what a 3 s power
// figure means.
constexpr uint32_t kPowerSampleMs = 250;
constexpr int kPowerWindow = 12;
uint16_t s_powerWindow[kPowerWindow] = {0};
int s_powerWindowNext = 0;
uint16_t s_powerHistory[kPowerHistoryLen] = {0};
int s_powerHistoryNewest = kPowerHistoryLen - 1;
uint32_t s_powerHistorySeq = 0;
uint32_t s_lastPowerSampleMs = 0;
int s_samplesSinceRecord = 0;

// Snapped to the step, so repeated 0.1 kg steps do not drift to 8.299999.
float clampToSpec(float value, const ProfileSpec& spec) {
  value = roundf(value / spec.step) * spec.step;
  return constrain(value, spec.min, spec.max);
}

void saveDevice(Preferences& store, const char* key, const SavedDevice& in) {
  if (!in.valid) {
    if (store.isKey(key)) store.remove(key);
    return;
  }
  uint8_t blob[7];
  memcpy(blob, in.addr, 6);
  blob[6] = in.type;
  store.putBytes(key, blob, sizeof(blob));
}

}  // namespace

void Settings::load() {
  xSemaphoreTake(prefsMutex(), portMAX_DELAY);
  prefs.begin("cydshift", true);
  gearCount   = prefs.getInt("gearCount", kDefaultGearCount);
  riderKg     = loadFloat(prefs, "riderKg", kDefaultRiderKg);
  bikeKg      = loadFloat(prefs, "bikeKg", kDefaultBikeKg);
  wheelMm     = loadFloat(prefs, "wheelMm", kDefaultWheelMm);
  riderCm     = loadFloat(prefs, "riderCm", kDefaultRiderCm);
  ftpW        = loadFloat(prefs, "ftpW", kDefaultFtpW);
  upChannel   = prefs.getInt("upCh", kDefaultUpChannel);
  downChannel = prefs.getInt("downCh", kDefaultDownChannel);
  gear        = prefs.getInt("gear", 0);
  cadenceSource = prefs.getUChar("cadSrc", (uint8_t)CadenceSource::Trainer) == 1
                      ? CadenceSource::Sensor
                      : CadenceSource::Trainer;
  loadDevice(prefs, "trainer", trainer);
  loadDevice(prefs, "shifter", shifter);
  loadDevice(prefs, "cadence", cadence);
  prefs.end();
  xSemaphoreGive(s_prefsMutex);

  if (gearCount < 2) gearCount = 2;
  if (gearCount > 36) gearCount = 36;
  if (gear < 1 || gear > gearCount) gear = neutralGear();
  for (int i = 0; i < kProfileFieldCount; i++) {
    float& value = profileSetting(*this, (ProfileField)i);
    value = clampToSpec(value, kProfileSpecs[i]);
  }
}

void Settings::save() {
  xSemaphoreTake(prefsMutex(), portMAX_DELAY);
  prefs.begin("cydshift", false);
  prefs.putInt("gearCount", gearCount);
  // Left over from the grade-offset gears.
  if (prefs.isKey("minOffset")) prefs.remove("minOffset");
  if (prefs.isKey("maxOffset")) prefs.remove("maxOffset");
  prefs.putFloat("riderKg", riderKg);
  prefs.putFloat("bikeKg", bikeKg);
  prefs.putFloat("wheelMm", wheelMm);
  prefs.putFloat("riderCm", riderCm);
  prefs.putFloat("ftpW", ftpW);
  prefs.putInt("upCh", upChannel);
  prefs.putInt("downCh", downChannel);
  prefs.putInt("gear", gear);
  prefs.putUChar("cadSrc", (uint8_t)cadenceSource);
  saveDevice(prefs, "trainer", trainer);
  saveDevice(prefs, "shifter", shifter);
  saveDevice(prefs, "cadence", cadence);
  prefs.end();
  xSemaphoreGive(s_prefsMutex);
}

float gearRatio(int gear) {
  if (g_cfg.gearCount < 2) return kRealChainring / kRealCog;
  if (gear < 1) gear = 1;
  if (gear > g_cfg.gearCount) gear = g_cfg.gearCount;
  if (g_cfg.gearCount == kGearCountLow) return kRealGearRatios[gear - 1];
  // Equal steps in ratio, so every shift adds the same amount - which makes
  // the low shifts big in % terms (+35 % from the bottom gear) and the top
  // ones small (+3 %).
  const float t = (float)(gear - 1) / (float)(g_cfg.gearCount - 1);
  return kSmoothMinRatio + (kSmoothMaxRatio - kSmoothMinRatio) * t;
}

float gearWheelMm(int gear) {
  return g_cfg.wheelMm * gearRatio(gear) / (kRealChainring / kRealCog);
}

Profile currentProfile() {
  Profile profile;
  for (int i = 0; i < kProfileFieldCount; i++) {
    profile.value[i] = profileSetting(g_cfg, (ProfileField)i);
  }
  return profile;
}

void stepProfile(Profile& profile, ProfileField field, int steps) {
  const ProfileSpec& spec = kProfileSpecs[(int)field];
  float& value = profile.value[(int)field];
  value = clampToSpec(value + spec.step * (float)steps, spec);
}

bool applyProfile(const Profile& profile) {
  bool changed = false;
  for (int i = 0; i < kProfileFieldCount; i++) {
    float& setting = profileSetting(g_cfg, (ProfileField)i);
    if (fabsf(setting - profile.value[i]) > 1e-3f) {
      setting = profile.value[i];
      changed = true;
    }
  }
  return changed;
}

void samplePowerHistory() {
  const uint32_t now = millis();
  if (s_lastPowerSampleMs != 0 && now - s_lastPowerSampleMs < kPowerSampleMs) return;
  s_lastPowerSampleMs = now;

  const int16_t watts = g_ride.power;
  s_powerWindow[s_powerWindowNext] = (uint16_t)max<int16_t>(0, watts);
  s_powerWindowNext = (s_powerWindowNext + 1) % kPowerWindow;

  if (++s_samplesSinceRecord < 1000 / kPowerSampleMs) return;
  s_samplesSinceRecord = 0;
  uint32_t sum = 0;
  for (uint16_t w : s_powerWindow) sum += w;
  s_powerHistoryNewest = (s_powerHistoryNewest + 1) % kPowerHistoryLen;
  s_powerHistory[s_powerHistoryNewest] = (uint16_t)(sum / kPowerWindow);
  s_powerHistorySeq++;
}

uint16_t powerHistory(int secondsAgo) {
  if (secondsAgo < 0 || secondsAgo >= kPowerHistoryLen) return 0;
  if ((uint32_t)secondsAgo >= s_powerHistorySeq) return 0;
  const int index = (s_powerHistoryNewest - secondsAgo + kPowerHistoryLen) % kPowerHistoryLen;
  return s_powerHistory[index];
}

uint32_t powerHistorySeq() { return s_powerHistorySeq; }

namespace {

int nearestGear(float ratio) {
  int best = 1;
  float bestDist = 1e9f;
  for (int g = 1; g <= g_cfg.gearCount; g++) {
    const float d = fabsf(logf(gearRatio(g) / ratio));
    if (d < bestDist) { bestDist = d; best = g; }
  }
  return best;
}

}  // namespace

int neutralGear() { return nearestGear(kRealChainring / kRealCog); }

void setGear(int gear) {
  if (gear < 1) gear = 1;
  if (gear > g_cfg.gearCount) gear = g_cfg.gearCount;
  g_cfg.gear = gear;
}

bool shiftUp() {
  if (g_cfg.gear >= g_cfg.gearCount) return false;
  g_cfg.gear++;
  return true;
}

bool shiftDown() {
  if (g_cfg.gear <= 1) return false;
  g_cfg.gear--;
  return true;
}

void toggleGearCount() {
  const float ratio = gearRatio(g_cfg.gear);
  g_cfg.gearCount = (g_cfg.gearCount == kGearCountLow) ? kGearCountHigh : kGearCountLow;
  setGear(nearestGear(ratio));
}

bool useSensorCadence() {
  if (g_cfg.cadenceSource != CadenceSource::Sensor) return false;
  if (!g_ride.sensorConnected) return false;
  // lastSensorDataMs starts at zero, so without this a freshly connected
  // sensor that has sent nothing reads as live for the first few seconds.
  if (!g_ride.sensorCrankValid) return false;
  if (elapsedSince(g_ride.lastSensorDataMs) < kCadenceTimeoutMs) return true;
  // Silent. Many sensors go quiet on purpose when the cranks stop, and falling
  // back then would hand the app the trainer's cadence - which can keep
  // reading while the flywheel spins down. Only treat silence as a dropout if
  // the trainer says the rider is still pedalling.
  return g_ride.trainerCadence == 0;
}

uint8_t effectiveCadence() {
  return useSensorCadence() ? g_ride.sensorCadence : g_ride.trainerCadence;
}

namespace {

// Allowed gap between crank revolutions before calling it stopped: two
// revolutions at the last cadence, but never below 1.3 s (a 1 Hz sensor can
// deliver a revolution up to a second late) nor above 2 s (30 rpm).
uint32_t stopAfterMs(uint8_t rpm) {
  if (rpm == 0) return 0;
  const uint32_t twoRevs = 2u * 60000u / rpm;
  return constrain(twoRevs, 1300u, 2000u);
}

}  // namespace

void expireStoppedCadence() {
  const uint8_t trainer = g_ride.trainerCadence;
  if (trainer != 0 && elapsedSince(g_ride.trainerRevMs) > stopAfterMs(trainer)) {
    g_ride.trainerCadence = 0;
  }
  const uint8_t sensor = g_ride.sensorCadence;
  if (sensor != 0 && elapsedSince(g_ride.sensorRevMs) > stopAfterMs(sensor)) {
    g_ride.sensorCadence = 0;
  }
}

void toggleCadenceSource() {
  g_cfg.cadenceSource = (g_cfg.cadenceSource == CadenceSource::Trainer)
                            ? CadenceSource::Sensor
                            : CadenceSource::Trainer;
}
