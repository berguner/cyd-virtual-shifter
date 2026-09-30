// Virtual shifting bridge for a Wahoo KICKR v5 on a Cheap Yellow Display.
//
// The KICKR v5 never got Wahoo's virtual-shifting firmware and does not speak
// FTMS, so neither Zwift nor MyWhoosh can shift gears on it. This sits in the
// middle: it talks Wahoo's own control protocol up to the trainer, presents a
// plain FTMS trainer down to the app, and shifts by changing the wheel size
// the trainer's sim physics uses. Gear changes come from the Di2 hood buttons
// over BLE.

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "CadenceSensor.h"
#include "Config.h"
#include "DeviceRegistry.h"
#include "Di2Client.h"
#include "KickrClient.h"
#include "State.h"
#include "TrainerServer.h"
#include "Ui.h"

namespace {

// The Wahoo control point has to be unlocked and then explicitly put into sim
// mode, with a settling gap in between. Staged across control passes so the
// wait does not block the loop.
enum class TrainerInit : uint8_t { Unlock, Settle, SimMode, AwaitSimMode, Ready };
TrainerInit s_init = TrainerInit::Unlock;
uint32_t s_unlockedAtMs = 0;
uint32_t s_simModeSentMs = 0;

// What the trainer was last told, as opposed to what the app is asking for.
bool s_trainerInErg = false;
float s_lastGradeSent = NAN;
float s_lastWheelSent = NAN;
uint32_t s_lastErgSent = 0xFFFFFFFF;

void restartTrainerInit() {
  s_init = TrainerInit::Unlock;
  s_trainerInErg = false;
  s_lastGradeSent = NAN;
  s_lastWheelSent = NAN;
  s_lastErgSent = 0xFFFFFFFF;
}

bool s_gearDirty = false;
uint32_t s_gearChangedMs = 0;

// An NVS commit stalls for hundreds of milliseconds, which in the loop shows
// up as a gap in the notifications going to the app. The link task already
// does the slow work, so saving happens there.
volatile bool s_settingsDirty = false;

// Set when the profile screen closes with new values. The weight only reaches
// the trainer through a sim-mode write, which would end ERG, so it waits for
// the sim path; a new wheel size is picked up there on its own.
bool s_profileChanged = false;

// Set from the UI, consumed by the link task - connecting blocks for seconds
// and has no business running on the thread that draws the screen.
volatile int s_pendingSelect = -1;
volatile int s_pendingForget = -1;

void markGearChanged() {
  s_gearDirty = true;
  s_gearChangedMs = millis();
  // Force the next control pass to push the new gear immediately.
  s_lastWheelSent = NAN;
}

// Runs on the NimBLE host task: only touch plain state here, never write BLE.
void onDi2Button(int channel, Di2::Press press) {
  if (press == Di2::Press::Long) return;
  const int steps = (press == Di2::Press::Double) ? 2 : 1;
  bool moved = false;
  for (int i = 0; i < steps; i++) {
    if (channel == g_cfg.upChannel) {
      moved |= shiftUp();
    } else if (channel == g_cfg.downChannel) {
      moved |= shiftDown();
    }
  }
  if (moved) markGearChanged();
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override { Registry::note(device); }
};

ScanCallbacks s_scanCallbacks;

bool forgetIfSaved(const Registry::Entry& entry) {
  if (g_cfg.trainer.matches(entry.addr)) {
    g_cfg.trainer.forget();
    Kickr::disconnect();
  } else if (g_cfg.shifter.matches(entry.addr)) {
    g_cfg.shifter.forget();
    Di2::disconnect();
  } else if (g_cfg.cadence.matches(entry.addr)) {
    g_cfg.cadence.forget();
    Sensor::disconnect();
  } else {
    return false;
  }
  g_cfg.save();
  Serial.printf("[select] forgot %s\n", entry.name);
  return true;
}

// Roles are confirmed by what the device actually exposes, not by what it
// called itself in the advertisement - each connectTo hangs up again if the
// service it needs is not there, so an unclassified device can just be tried
// against each role in turn.
void assignDevice(const Registry::Entry& entry) {
  if (g_cfg.trainer.matches(entry.addr) || g_cfg.shifter.matches(entry.addr) ||
      g_cfg.cadence.matches(entry.addr)) {
    Serial.printf("[select] %s is already chosen - hold the row to forget it\n", entry.name);
    return;
  }

  if (entry.role == Registry::Role::HeartRate) {
    // Probing would open a real link and take the strap's single connection
    // slot away from the training app for nothing.
    Serial.printf("[select] %s is a heart rate strap - pair it with the app\n", entry.name);
    return;
  }

  const NimBLEAddress addr = Registry::addressOf(entry);
  const bool unknown = entry.role == Registry::Role::Unknown;
  const uint32_t age = Registry::ageMs(entry);
  Serial.printf("[select] probing %s at %s type %u, last heard %lu ms ago\n", entry.name,
                addr.toString().c_str(), (unsigned)entry.addrType, (unsigned long)age);

  // Connecting to a row that stopped advertising can only burn the connect
  // timeout, and with three roles to try that wedges the link task for the
  // best part of a minute.
  if (age > Registry::kStaleAfterMs) {
    Serial.println("[select] that device is not advertising right now - wake it and retry");
    return;
  }

  if ((unknown || entry.role == Registry::Role::Trainer) && !Kickr::connected()) {
    if (Kickr::connectTo(addr)) {
      g_cfg.trainer.set(entry.addr, entry.addrType);
      g_cfg.save();
      restartTrainerInit();
      Serial.printf("[select] %s is the trainer\n", entry.name);
      return;
    }
  }
  if ((unknown || entry.role == Registry::Role::Shifter) && !Di2::connected()) {
    if (Di2::connectTo(addr)) {
      g_cfg.shifter.set(entry.addr, entry.addrType);
      g_cfg.save();
      Serial.printf("[select] %s is the shifter\n", entry.name);
      return;
    }
  }
  if ((unknown || entry.role == Registry::Role::Cadence) && !Sensor::connected()) {
    if (Sensor::connectTo(addr)) {
      g_cfg.cadence.set(entry.addr, entry.addrType);
      g_cfg.save();
      Serial.printf("[select] %s is the cadence sensor\n", entry.name);
      return;
    }
  }
  Serial.printf("[select] %s exposes nothing we can use\n", entry.name);
}

void reconnectSaved(NimBLEScan* scan) {
  Registry::Entry entry;

  if (g_cfg.trainer.valid && !Kickr::connected() &&
      Registry::seenRecently(g_cfg.trainer.addr, 15000, entry)) {
    if (scan->isScanning()) scan->stop();
    if (Kickr::connectTo(Registry::addressOf(entry))) restartTrainerInit();
  }
  if (g_cfg.shifter.valid && !Di2::connected() &&
      Registry::seenRecently(g_cfg.shifter.addr, 15000, entry)) {
    if (scan->isScanning()) scan->stop();
    Di2::connectTo(Registry::addressOf(entry));
  }
  if (g_cfg.cadence.valid && !Sensor::connected() &&
      Registry::seenRecently(g_cfg.cadence.addr, 15000, entry)) {
    if (scan->isScanning()) scan->stop();
    Sensor::connectTo(Registry::addressOf(entry));
  }
}

void linkTask(void* unused) {
  (void)unused;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&s_scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(50);

  for (;;) {
    Kickr::dropIfStale();
    Di2::dropIfStale();
    Sensor::dropIfStale();

    const int selection = s_pendingSelect;
    if (selection >= 0) {
      s_pendingSelect = -1;
      Registry::Entry entry;
      if (Registry::get((size_t)selection, entry)) {
        if (scan->isScanning()) scan->stop();
        assignDevice(entry);
      }
    }

    const int forget = s_pendingForget;
    if (forget >= 0) {
      s_pendingForget = -1;
      Registry::Entry entry;
      if (Registry::get((size_t)forget, entry) && !forgetIfSaved(entry)) {
        Serial.printf("[select] %s was not chosen, nothing to forget\n", entry.name);
      }
    }

    reconnectSaved(scan);
    AppLink::ensureAdvertising();

    if (s_settingsDirty) {
      s_settingsDirty = false;
      g_cfg.save();
    }

    // Keep the radio listening while the picker is open, or while anything the
    // rider chose is still missing.
    const bool wantScan = Ui::screen() == Ui::Screen::Devices ||
                          (g_cfg.trainer.valid && !Kickr::connected()) ||
                          (g_cfg.shifter.valid && !Di2::connected()) ||
                          (g_cfg.cadence.valid && !Sensor::connected());
    if (wantScan) {
      if (!scan->isScanning()) scan->start(0, false, true);
    } else if (scan->isScanning()) {
      scan->stop();
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void applyTrainerState() {
  if (!Kickr::connected()) {
    restartTrainerInit();
    return;
  }

  const AppLink::SimParams sim = AppLink::simParams();
  const float crr = sim.crr > 0.0f ? sim.crr : kDefaultCrr;
  const float cw = sim.cw > 0.0f ? sim.cw : kDefaultCw;
  const float weight = g_cfg.riderKg + g_cfg.bikeKg;

  switch (s_init) {
    case TrainerInit::Unlock:
      if (Kickr::unlock()) {
        s_unlockedAtMs = millis();
        s_init = TrainerInit::Settle;
      }
      return;
    case TrainerInit::Settle:
      if (millis() - s_unlockedAtMs >= kUnlockSettleMs) s_init = TrainerInit::SimMode;
      return;
    case TrainerInit::SimMode:
      // The wheel size is also how gears are applied, and it persists on the
      // trainer between sessions - left over from another app, it silently
      // scaled every watt. Always set it here rather than trust what is there.
      if (Kickr::setWheelCircumference(gearWheelMm(g_cfg.gear))) {
        s_lastWheelSent = gearWheelMm(g_cfg.gear);
      }
      if (Kickr::setSimMode(weight, crr, cw)) {
        s_simModeSentMs = millis();
        s_init = TrainerInit::AwaitSimMode;
      }
      return;
    case TrainerInit::AwaitSimMode:
      // The write going through does not mean the trainer took it: once it
      // never answered, and it stalled in ERG straight after. Resend until it
      // says yes.
      if (Kickr::acknowledged(kWahooSetSimMode)) {
        s_init = TrainerInit::Ready;
        s_trainerInErg = false;
        s_lastGradeSent = NAN;
        s_lastErgSent = 0xFFFFFFFF;
        Serial.println("[kickr] unlocked and in sim mode");
      } else if (elapsedSince(s_simModeSentMs) > kKickrAckTimeoutMs) {
        Serial.println("[kickr] sim mode not acknowledged - resending");
        s_init = TrainerInit::SimMode;
      }
      return;
    case TrainerInit::Ready:
      break;
  }

  // Never into ERG from a standstill. A KICKR v5 put into ERG with its flywheel
  // stopped went on reporting zero speed and power every time it was in ERG
  // afterwards, braking as if the rider had stalled, until power-cycled. So ERG
  // waits until the rider is pedalling with the flywheel turning, and gives way
  // to sim once they stop. Once in, cadence alone keeps it: the crank counters
  // kept working while wheel speed was frozen. The grace period covers a rider
  // who stops pedalling for a moment, say to tap ERG on in the app.
  const bool pedalling = effectiveCadence() > 0;
  static bool s_wasPedalling = false;
  static uint32_t s_stoppedAtMs = 0;
  if (!pedalling && s_wasPedalling) s_stoppedAtMs = millis();
  s_wasPedalling = pedalling;
  const bool stopped = !pedalling && elapsedSince(s_stoppedAtMs) >= kErgStopGraceMs;
  const bool ergReady =
      s_trainerInErg ? !stopped : (pedalling && g_ride.speedKph >= kErgMinEntryKph);
  static bool s_heldOffLogged = false;
  if (g_ride.ergMode && !ergReady) {
    if (!s_heldOffLogged) {
      Serial.printf("[kickr] ERG %u W waiting for the rider to pedal (%u rpm, %.1f kph)\n",
                    (unsigned)g_ride.targetPower, (unsigned)effectiveCadence(),
                    (float)g_ride.speedKph);
      s_heldOffLogged = true;
    }
  } else {
    s_heldOffLogged = false;
  }

  if (g_ride.ergMode && ergReady) {
    // Workout blocks own the resistance; gears would only fight the target.
    if (g_ride.targetPower != s_lastErgSent) {
      if (Kickr::setErgMode(g_ride.targetPower)) {
        s_lastErgSent = g_ride.targetPower;
        s_trainerInErg = true;
      }
    }
    s_lastGradeSent = NAN;
    return;
  }

  // A sim grade only sets a parameter - it does not take the trainer back out
  // of ERG, which would otherwise hold the last workout wattage for the rest of
  // the ride. qdomyos-zwift re-runs its whole init here, so do the same.
  if (s_trainerInErg) {
    Serial.println("[kickr] leaving ERG, re-establishing sim mode");
    restartTrainerInit();
    return;
  }

  AppLink::SimParams fresh;
  const bool appChangedSim = AppLink::takeSimParams(fresh);
  if (appChangedSim || s_profileChanged) {
    s_profileChanged = false;
    Serial.printf("[kickr] setSimMode kg %.1f crr %.4f cw %.2f (app wind %.2f m/s)\n", weight,
                  fresh.crr > 0.0f ? fresh.crr : kDefaultCrr, fresh.cw > 0.0f ? fresh.cw : kDefaultCw,
                  fresh.windMps);
    Kickr::setSimMode(weight, fresh.crr > 0.0f ? fresh.crr : kDefaultCrr,
                      fresh.cw > 0.0f ? fresh.cw : kDefaultCw);
    // Do not assume the trainer kept its grade across a sim-mode write; the
    // check below only resends on a change, so a reset would stick.
    s_lastGradeSent = NAN;
  }

  const float wheel = gearWheelMm(g_cfg.gear);
  if (isnan(s_lastWheelSent) || fabsf(wheel - s_lastWheelSent) > 0.5f) {
    if (Kickr::setWheelCircumference(wheel)) {
      Serial.printf("[kickr] gear %d/%d ratio %.2f -> wheel %.0f mm\n", g_cfg.gear,
                    g_cfg.gearCount, gearRatio(g_cfg.gear), wheel);
      s_lastWheelSent = wheel;
    }
  }

  // constrain() is a macro that evaluates its first argument up to three times,
  // and appGrade is volatile - so read it once into a local.
  const float requested = g_ride.appGrade;
  const float effective = constrain(requested, kGradeClampLow, kGradeClampHigh);
  if (isnan(s_lastGradeSent) || fabsf(effective - s_lastGradeSent) > 0.05f) {
    if (Kickr::setSimGrade(effective)) {
      s_lastGradeSent = effective;
      g_ride.appliedGrade = effective;
    }
  }
  s_lastErgSent = 0xFFFFFFFF;
}

void handleTouch() {
  const Ui::Touch touch = Ui::pollTouch();
  switch (touch.action) {
    case Ui::Action::ShiftUp:
      if (shiftUp()) markGearChanged();
      break;
    case Ui::Action::ShiftDown:
      if (shiftDown()) markGearChanged();
      break;
    case Ui::Action::ToggleCadenceSource:
      toggleCadenceSource();
      s_settingsDirty = true;
      Serial.printf("[cadence] source is now %s\n",
                    g_cfg.cadenceSource == CadenceSource::Sensor ? "sensor" : "trainer");
      break;
    case Ui::Action::ToggleGearCount:
      toggleGearCount();
      markGearChanged();
      s_settingsDirty = true;
      Serial.printf("[gears] %d gears, now in gear %d\n", g_cfg.gearCount, g_cfg.gear);
      break;
    case Ui::Action::OpenDevices:
      Ui::setScreen(Ui::Screen::Devices);
      break;
    case Ui::Action::OpenDebug:
      Ui::setScreen(Ui::Screen::Debug);
      break;
    case Ui::Action::OpenProfile:
      Ui::setScreen(Ui::Screen::Profile);
      break;
    case Ui::Action::CloseProfile:
      if (applyProfile(Ui::profileDraft())) {
        s_settingsDirty = true;
        s_profileChanged = true;
        Serial.printf("[profile] rider %.1f kg, bike %.1f kg, wheel %.0f mm, height %.0f cm\n",
                      g_cfg.riderKg, g_cfg.bikeKg, g_cfg.wheelMm, g_cfg.riderCm);
      }
      Ui::setScreen(Ui::Screen::Ride);
      break;
    case Ui::Action::StartRide:
      Ui::setScreen(Ui::Screen::Ride);
      break;
    case Ui::Action::SelectDevice:
      if (touch.index >= 0) s_pendingSelect = touch.index;
      break;
    case Ui::Action::ForgetDevice:
      if (touch.index >= 0) s_pendingForget = touch.index;
      break;
    case Ui::Action::ScrollUp:
    case Ui::Action::ScrollDown:
    case Ui::Action::None:
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[boot] CYD virtual shifter");

  g_cfg.load();
  if (g_cfg.gear < 1 || g_cfg.gear > g_cfg.gearCount) setGear(neutralGear());

  Ui::begin();

  NimBLEDevice::init(kServerName);
  NimBLEDevice::setMTU(255);

  AppLink::begin();
  Di2::setHandler(onDi2Button);

  xTaskCreatePinnedToCore(linkTask, "link", 5120, nullptr, 1, nullptr, 1);

  // Straight to the ride screen once a trainer has been picked; the links come
  // up in the background. The picker is always a tap on the header away.
  Ui::setScreen(g_cfg.trainer.valid ? Ui::Screen::Ride : Ui::Screen::Devices);

  Serial.printf("[boot] %d gears, ratio %.2f to %.2f, gear %d\n", g_cfg.gearCount, gearRatio(1),
                gearRatio(g_cfg.gearCount), g_cfg.gear);
}

void loop() {
  const uint32_t workStartUs = micros();
  const uint32_t now = millis();
  static uint32_t lastControl = 0;
  static uint32_t lastPublish = 0;
  static uint32_t lastPerf = 0;
  static uint32_t loopCount = 0;
  static uint32_t maxWorkUs = 0;
  static uint64_t totalWorkUs = 0;
  handleTouch();

  AppLink::flushControlPointResponse();

  // Polled fast on purpose: this is the one piece of shift latency we own.
  // The call is nearly free when nothing moved - it only writes to the trainer
  // when the target actually changed.
  if (now - lastControl >= 50) {
    lastControl = now;
    applyTrainerState();
  }

  if (now - lastPublish >= 500) {
    lastPublish = now;
    AppLink::publishIndoorBikeData();
  }

  // The KICKR stops notifying when the flywheel stops; zero things out so the
  // display does not freeze on the last sample. Generous, because reading zero
  // while someone is pedalling is worse than briefly holding a stale number.
  const uint32_t trainerSilence = elapsedSince(g_ride.lastTrainerDataMs);
  if (g_ride.kickrConnected && trainerSilence > kTrainerSilenceMs) {
    if (g_ride.power != 0 || g_ride.trainerCadence != 0) {
      Serial.printf("[kickr] silent for %lu ms - zeroing power\n",
                    (unsigned long)trainerSilence);
    }
    g_ride.power = 0;
    g_ride.trainerCadence = 0;
    g_ride.speedKph = 0.0f;
  }

  expireStoppedCadence();
  samplePowerHistory();

  // One line a second with everything that decides the resistance, so a ride
  // log can be lined up against how it felt.
  static uint32_t lastRideLog = 0;
  if (now - lastRideLog >= 1000) {
    lastRideLog = now;
    const AppLink::SimParams sim = AppLink::simParams();
    Serial.printf("[ride] app %+.2f%% sent %+.2f%% gear %d/%d ratio %.2f | crr %.4f cw %.2f"
                  " wind %.2f | erg %d tgt %u | %dW %urpm %.1fkph | cad %s sensor %u (pkt %lu"
                  " rev %lu ms ago) trainer %u (rev %lu ms ago)\n",
                  (float)g_ride.appGrade, (float)g_ride.appliedGrade, g_cfg.gear, g_cfg.gearCount,
                  gearRatio(g_cfg.gear), sim.crr, sim.cw, sim.windMps, (int)g_ride.ergMode,
                  (unsigned)g_ride.targetPower, (int)g_ride.power, (unsigned)effectiveCadence(),
                  (float)g_ride.speedKph, useSensorCadence() ? "sensor" : "trainer",
                  (unsigned)g_ride.sensorCadence,
                  (unsigned long)elapsedSince(g_ride.lastSensorDataMs),
                  (unsigned long)elapsedSince(g_ride.sensorRevMs), (unsigned)g_ride.trainerCadence,
                  (unsigned long)elapsedSince(g_ride.trainerRevMs));
  }

  if (s_gearDirty && now - s_gearChangedMs > 5000) {
    s_gearDirty = false;
    s_settingsDirty = true;
  }

  Ui::update();

  // Everything above is the actual work; the delay below is idle time. Print
  // the real cost every 10 s so the load can be read off a running board
  // instead of guessed at.
  const uint32_t workUs = micros() - workStartUs;
  if (workUs > maxWorkUs) maxWorkUs = workUs;
  totalWorkUs += workUs;
  loopCount++;
  if (now - lastPerf >= 10000 && loopCount > 0) {
    Serial.printf("[perf] heap %u (min %u)  loops/s %u  work avg/max %u/%u us  duty %u%%\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                  (unsigned)(loopCount / 10), (unsigned)(totalWorkUs / loopCount),
                  (unsigned)maxWorkUs, (unsigned)(totalWorkUs / 100000));
    lastPerf = now;
    loopCount = 0;
    maxWorkUs = 0;
    totalWorkUs = 0;
  }

  delay(10);
}
