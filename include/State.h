#pragma once
#include <Arduino.h>
#include "Config.h"

// Live telemetry, written by BLE callbacks and read by the UI. Every field is
// a single word, so plain volatile reads are good enough here - nothing
// downstream needs two fields to agree with each other.
enum class CadenceSource : uint8_t { Trainer = 0, Sensor = 1 };

struct RideState {
  volatile int16_t  power = 0;
  volatile uint8_t  trainerCadence = 0;
  volatile float    speedKph = 0.0f;
  volatile uint32_t lastTrainerDataMs = 0;
  volatile uint32_t trainerRevMs = 0;  // when the crank count last moved

  // External crank sensor. The raw revolution counters are kept so they can be
  // spliced into the power packet the app sees, rather than re-derived.
  volatile uint8_t  sensorCadence = 0;
  volatile uint16_t sensorCrankRev = 0;
  volatile uint16_t sensorCrankTime = 0;
  volatile bool     sensorCrankValid = false;
  volatile uint32_t lastSensorDataMs = 0;
  volatile uint32_t sensorRevMs = 0;   // when the crank count last moved
  volatile bool     sensorConnected = false;

  volatile float    appGrade = 0.0f;     // what the app asked for
  volatile float    appliedGrade = 0.0f; // what we actually sent the trainer
  volatile uint16_t targetPower = 0;
  volatile bool     ergMode = false;

  volatile bool kickrConnected = false;
  volatile bool di2Connected = false;
  volatile bool appConnected = false;
};

// A device the rider picked on the setup screen. Both the address bytes and
// the address type are kept: a random static address will not connect if it is
// reconstructed as a public one.
struct SavedDevice {
  uint8_t addr[6] = {0};
  uint8_t type = 0;
  bool valid = false;

  bool matches(const uint8_t other[6]) const { return valid && memcmp(addr, other, 6) == 0; }
  void set(const uint8_t other[6], uint8_t otherType) {
    memcpy(addr, other, 6);
    type = otherType;
    valid = true;
  }
  void forget() { valid = false; }
};

// Persisted in NVS so a power cycle keeps your gear range and channel mapping.
struct Settings {
  int   gearCount   = kDefaultGearCount;
  float riderKg     = kDefaultRiderKg;
  float bikeKg      = kDefaultBikeKg;
  float wheelMm     = kDefaultWheelMm;
  float riderCm     = kDefaultRiderCm;
  float ftpW        = kDefaultFtpW;
  int   upChannel   = kDefaultUpChannel;
  int   downChannel = kDefaultDownChannel;
  int   gear        = 1;
  CadenceSource cadenceSource = CadenceSource::Trainer;

  SavedDevice trainer;
  SavedDevice shifter;
  SavedDevice cadence;

  void load();
  void save();
};

extern RideState g_ride;
extern Settings  g_cfg;

// ---------------------------------------------------------------------------
// Gears
// ---------------------------------------------------------------------------

// Virtual gear ratio (chainring / cog) for the given 1-based gear.
float gearRatio(int gear);

// Wheel size to give the trainer so it rides like the given gear.
float gearWheelMm(int gear);

// Gear nearest the real chainring and cog - the one that rides like no bridge
// is in the way at all. Used as the power-on default.
int neutralGear();

bool shiftUp();
bool shiftDown();
void setGear(int gear);

// Switches between the two supported gear counts, landing on the gear nearest
// the current ratio so the resistance does not jump - the two cover different
// ranges, so the same position in each would not be the same gear.
void toggleGearCount();

// ---------------------------------------------------------------------------
// Rider profile, edited on the profile screen
// ---------------------------------------------------------------------------

enum class ProfileField : uint8_t { RiderKg, BikeKg, FtpW, WheelMm, RiderCm };
constexpr int kProfileFieldCount = 5;

struct Profile {
  float value[kProfileFieldCount];
};

Profile currentProfile();

// Moves a field by whole steps of its own size, clamped to its range.
void stepProfile(Profile& profile, ProfileField field, int steps);

// Copies the profile into the settings; true if anything changed.
bool applyProfile(const Profile& profile);

// ---------------------------------------------------------------------------
// Power history, for the ride screen's graph
// ---------------------------------------------------------------------------

// One entry per second, each the 3 s moving average of power at that moment.
constexpr int kPowerHistoryLen = 316;

// Call from the loop; it keeps its own time.
void samplePowerHistory();

// Watts `secondsAgo` entries back (0 = newest); 0 beyond what is recorded.
uint16_t powerHistory(int secondsAgo);

// Goes up by one per new entry, so a reader can tell when to redraw.
uint32_t powerHistorySeq();

// ---------------------------------------------------------------------------
// Cadence
// ---------------------------------------------------------------------------

// True when the external sensor is both selected and actually delivering. If
// it is selected but silent we fall back to the trainer rather than dropping
// cadence mid-ride - the UI shows the fallback instead of hiding it.
bool useSensorCadence();

uint8_t effectiveCadence();

// Zeroes a cadence whose cranks have stopped turning. Runs from the loop, not
// the BLE callbacks: a sensor often stops notifying altogether when the cranks
// stop, so waiting for the next packet to notice could mean waiting forever.
void expireStoppedCadence();

void toggleCadenceSource();
