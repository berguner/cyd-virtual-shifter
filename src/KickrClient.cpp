#include "KickrClient.h"

#include "BleLink.h"
#include "Config.h"
#include "State.h"
#include "TrainerServer.h"

namespace {

NimBLEClient* s_client = nullptr;
DisconnectGate s_gate;
NimBLERemoteCharacteristic* s_control = nullptr;

// Cycling Power Measurement bookkeeping. The KICKR only advances the event
// timestamps when a revolution actually happens, so a frozen pair of samples
// means "stopped", not "same as last time".
uint16_t s_lastCrankRev = 0;
uint16_t s_lastCrankTime = 0;
bool     s_haveCrank = false;

uint32_t s_lastWheelRev = 0;
uint16_t s_lastWheelTime = 0;
bool     s_haveWheel = false;
uint32_t s_wheelChangedMs = 0;

// Written from the NimBLE task (packets, replies) and the loop (commands).
Kickr::Debug s_dbg;
portMUX_TYPE s_dbgLock = portMUX_INITIALIZER_UNLOCKED;

template <typename F>
void recordDebug(F update) {
  portENTER_CRITICAL(&s_dbgLock);
  update(s_dbg);
  portEXIT_CRITICAL(&s_dbgLock);
}

// Per command: how many writes went out, and the write count the trainer's
// latest answer belongs to - so an answer to an earlier write never counts for
// the current one. Guarded by s_dbgLock.
struct Exchange {
  uint32_t sent = 0;
  uint32_t answered = 0;
  uint8_t status = 0;
};
Exchange s_exchanges[5];

Exchange* exchangeFor(uint8_t op) {
  switch (op) {
    case kWahooUnlock: return &s_exchanges[0];
    case kWahooSetSimMode: return &s_exchanges[1];
    case kWahooSetSimGrade: return &s_exchanges[2];
    case kWahooSetWheelCircum: return &s_exchanges[3];
    case kWahooSetErgMode: return &s_exchanges[4];
    default: return nullptr;
  }
}

// Everything the trainer was telling us is stale the moment the link drops:
// leave it set and publishIndoorBikeData keeps notifying the last wattage for
// the rest of the ride.
void resetTelemetry() {
  s_haveCrank = false;
  s_haveWheel = false;
  g_ride.power = 0;
  g_ride.trainerCadence = 0;
  g_ride.speedKph = 0.0f;
}

// Speed is not used for anything the rider sees - the apps model that from
// power - but the FTMS packet is positional, and leaving the field out shifts
// everything after it for any app that does not honour the More Data flag.
void updateSpeed(uint32_t rev, uint16_t eventTime) {
  const uint32_t now = millis();
  if (s_haveWheel) {
    const uint32_t dRev = rev - s_lastWheelRev;
    const uint16_t dT = (uint16_t)(eventTime - s_lastWheelTime);
    if (dT > 0) {
      const float seconds = (float)dT / 2048.0f;  // wheel events tick at 1/2048 s
      const float metres = (float)dRev * (g_cfg.wheelMm / 1000.0f);
      const float kph = (metres / seconds) * 3.6f;
      if (kph >= 0.0f && kph < 150.0f) g_ride.speedKph = kph;
      s_wheelChangedMs = now;
    } else if (now - s_wheelChangedMs > 3000) {
      g_ride.speedKph = 0.0f;
    }
  } else {
    s_wheelChangedMs = now;
    s_haveWheel = true;
  }
  s_lastWheelRev = rev;
  s_lastWheelTime = eventTime;
}

void updateCadence(uint16_t rev, uint16_t eventTime) {
  if (s_haveCrank) {
    const uint16_t dRev = (uint16_t)(rev - s_lastCrankRev);
    const uint16_t dT = (uint16_t)(eventTime - s_lastCrankTime);
    // Stopping is noticed by expireStoppedCadence(), on the same rule as the
    // sensor.
    if (dRev > 0 && dT > 0) {
      const float rpm = (float)dRev * 60.0f * 1024.0f / (float)dT;
      if (rpm < 250.0f) g_ride.trainerCadence = (uint8_t)lroundf(rpm);
      g_ride.trainerRevMs = millis();
    }
  } else {
    s_haveCrank = true;
  }
  s_lastCrankRev = rev;
  s_lastCrankTime = eventTime;
}

void onMeasurement(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  (void)chr;
  (void)isNotify;
  if (len < 4) return;

  // A gap here is the only way a pedalling rider can read as zero watts, so
  // make it visible rather than inferring it from the symptom.
  const uint32_t now = millis();
  if (g_ride.lastTrainerDataMs != 0) {
    const uint32_t gap = elapsedSince(g_ride.lastTrainerDataMs);
    if (gap > 2000) Serial.printf("[kickr] notification gap %lu ms\n", (unsigned long)gap);
  }

  const uint16_t flags = (uint16_t)(data[0] | (data[1] << 8));
  g_ride.power = (int16_t)(data[2] | (data[3] << 8));
  g_ride.lastTrainerDataMs = now;

  size_t i = 4;
  bool haveTorque = false;
  uint16_t torque = 0;
  bool haveWheel = false;
  uint32_t wheelRev = 0;
  uint16_t wheelTime = 0;
  bool haveCrank = false;
  uint16_t crankRev = 0;
  uint16_t crankTime = 0;

  if (flags & 0x0001) i += 1;  // pedal power balance
  if (flags & 0x0004) {        // accumulated torque
    if (len >= i + 2) {
      torque = (uint16_t)(data[i] | (data[i + 1] << 8));
      haveTorque = true;
    }
    i += 2;
  }
  if (flags & 0x0010) {  // wheel revolution data
    if (len >= i + 6) {
      wheelRev = (uint32_t)data[i] | ((uint32_t)data[i + 1] << 8) |
                 ((uint32_t)data[i + 2] << 16) | ((uint32_t)data[i + 3] << 24);
      wheelTime = (uint16_t)(data[i + 4] | (data[i + 5] << 8));
      haveWheel = true;
      updateSpeed(wheelRev, wheelTime);
    }
    i += 6;
  }
  if (flags & 0x0020) {        // crank revolution data
    if (len >= i + 4) {
      crankRev = (uint16_t)(data[i] | (data[i + 1] << 8));
      crankTime = (uint16_t)(data[i + 2] | (data[i + 3] << 8));
      haveCrank = true;
      updateCadence(crankRev, crankTime);
    }
    i += 4;
  }

  recordDebug([&](Kickr::Debug& d) {
    d.flags = flags;
    d.power = g_ride.power;
    if (haveTorque) d.torque = torque;
    if (haveWheel) {
      d.wheelRevs = wheelRev;
      d.wheelTime = wheelTime;
    }
    if (haveCrank) {
      d.crankRevs = crankRev;
      d.crankTime = crankTime;
    }
    d.packets++;
    d.lastPacketMs = now;
  });

  // Once a second, show the packet and what came out of it. Nothing else in
  // the log reports the actual numbers, so a zero on screen is otherwise
  // indistinguishable from a trainer that is genuinely idle.
  static uint32_t lastDump = 0;
  if (now - lastDump >= 1000) {
    lastDump = now;
    char hex[80];
    size_t n = 0;
    for (size_t b = 0; b < len && n + 3 < sizeof(hex); b++) {
      n += snprintf(hex + n, sizeof(hex) - n, "%02x ", data[b]);
    }
    Serial.printf("[kickr] cps %s| flags %04x power %d cad %u speed %.1f\n", hex, flags,
                  (int)g_ride.power, (unsigned)g_ride.trainerCadence, g_ride.speedKph);
  }

  // Hand the untouched packet straight to the app: no re-encoding, so the
  // power and cadence Zwift records are exactly what the KICKR measured.
  AppLink::relayCyclingPower(data, len);
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    (void)client;
    Serial.println("[kickr] connected");
  }
  void onDisconnect(NimBLEClient* client, int reason) override {
    (void)client;
    Serial.printf("[kickr] disconnected, reason %d\n", reason);
    s_gate.signal();
    s_control = nullptr;
    g_ride.kickrConnected = false;
    resetTelemetry();
  }
};

ClientCallbacks s_callbacks;

// The KICKR answers every control write with 0x01 <opcode> <status> ...; a
// status other than 0x01 means it refused the command, which a successful BLE
// write alone does not tell us.
void onControlResponse(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  (void)chr;
  (void)isNotify;
  char hex[64];
  size_t n = 0;
  for (size_t b = 0; b < len && n + 3 < sizeof(hex); b++) {
    n += snprintf(hex + n, sizeof(hex) - n, "%02x ", data[b]);
  }
  const bool ok = len >= 3 && data[0] == 0x01 && data[2] == 0x01;
  // Grade replies arrive every second while riding; only surface refusals and
  // the one-off setup commands.
  if (!ok || (len >= 2 && data[1] != kWahooSetSimGrade)) {
    Serial.printf("[kickr] reply %s%s\n", hex, ok ? "" : "<- REFUSED");
  }

  if (len >= 3 && data[0] == 0x01) {
    const uint8_t op = data[1];
    const uint8_t status = data[2];
    Exchange* exchange = exchangeFor(op);
    recordDebug([op, status, exchange](Kickr::Debug& d) {
      if (exchange != nullptr) {
        exchange->answered = exchange->sent;
        exchange->status = status;
      }
      switch (op) {
        case kWahooUnlock: d.ackUnlock = status; break;
        case kWahooSetSimMode: d.ackSim = status; break;
        case kWahooSetSimGrade: d.ackGrade = status; break;
        case kWahooSetWheelCircum: d.ackWheel = status; break;
        case kWahooSetErgMode: d.ackErg = status; break;
        default: break;
      }
    });
  }
}

bool write(const uint8_t* data, size_t len, const char* what) {
  if (s_control == nullptr) return false;
  // Counted before writing: the answer can arrive before writeValue returns.
  Exchange* exchange = exchangeFor(data[0]);
  if (exchange != nullptr) recordDebug([exchange](Kickr::Debug&) { exchange->sent++; });
  if (!s_control->writeValue(data, len, true)) {
    Serial.printf("[kickr] write failed: %s\n", what);
    return false;
  }
  return true;
}

uint16_t clampToU16(float v) {
  if (v < 0.0f) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

// Hangs up and waits for the link to be fully torn down, so the next role
// probe is not rejected against a connection that is still closing.
void hangUp() {
  if (s_client == nullptr || !s_client->isConnected()) return;
  s_gate.arm();
  s_client->disconnect();
  s_gate.wait();
}

}  // namespace

namespace Kickr {

bool connected() {
  return s_client != nullptr && s_client->isConnected() && s_control != nullptr;
}

bool connectTo(const NimBLEAddress& addr) {
  if (s_client == nullptr) {
    s_client = NimBLEDevice::createClient();
    s_client->setClientCallbacks(&s_callbacks, false);
    s_client->setConnectTimeout(8000);
    s_client->setConnectionParams(12, 24, 0, 400);
  }
  if (!s_client->connect(addr)) {
    Serial.println("[kickr] connect failed");
    return false;
  }

  NimBLERemoteService* cps = s_client->getService(NimBLEUUID(UUID_SVC_CYCLING_POWER));
  if (cps == nullptr) {
    Serial.println("[kickr] no cycling power service");
    hangUp();
    return false;
  }

  // The Wahoo control point sits inside the Cycling Power Service on the v5.
  s_control = cps->getCharacteristic(NimBLEUUID(UUID_CHR_WAHOO_CONTROL));
  if (s_control == nullptr) {
    Serial.println("[kickr] no wahoo control point - is this really a KICKR?");
    hangUp();
    return false;
  }

  NimBLERemoteCharacteristic* measurement = cps->getCharacteristic(NimBLEUUID(UUID_CHR_CP_MEASUREMENT));
  if (measurement == nullptr || !measurement->canNotify()) {
    Serial.println("[kickr] no power measurement notifications");
    hangUp();
    return false;
  }
  // Before subscribing, so nothing from this link is wiped. Replies and
  // counters from a previous link would only mislead.
  recordDebug([](Kickr::Debug& d) {
    d = Kickr::Debug();
    for (Exchange& e : s_exchanges) e = Exchange();
  });
  measurement->subscribe(true, onMeasurement);
  if (s_control->canIndicate() || s_control->canNotify()) {
    s_control->subscribe(s_control->canNotify(), onControlResponse);
  }

  resetTelemetry();
  g_ride.kickrConnected = true;
  return true;
}

void dropIfStale() {
  if (s_client != nullptr && !s_client->isConnected()) {
    g_ride.kickrConnected = false;
    s_control = nullptr;
  }
}

bool unlock() {
  const uint8_t cmd[] = {kWahooUnlock, 0xEE, 0xFC};
  return write(cmd, sizeof(cmd), "unlock");
}

bool setSimMode(float weightKg, float crr, float cw) {
  const uint16_t w = clampToU16(constrain(weightKg, 0.0f, 655.0f) * 100.0f);
  const uint16_t r = clampToU16(constrain(crr, 0.0f, 65.0f) * 1000.0f);
  const uint16_t d = clampToU16(constrain(cw, 0.0f, 65.0f) * 1000.0f);
  const uint8_t cmd[] = {kWahooSetSimMode,
                         (uint8_t)(w & 0xFF), (uint8_t)(w >> 8),
                         (uint8_t)(r & 0xFF), (uint8_t)(r >> 8),
                         (uint8_t)(d & 0xFF), (uint8_t)(d >> 8)};
  if (!write(cmd, sizeof(cmd), "setSimMode")) return false;
  recordDebug([&](Kickr::Debug& dbg) {
    dbg.simKg = w / 100.0f;
    dbg.simCrr = r / 1000.0f;
    dbg.simCw = d / 1000.0f;
    dbg.mode = kWahooSetSimMode;
  });
  return true;
}

bool setSimGrade(float gradePercent) {
  const float fraction = constrain(gradePercent / 100.0f, -1.0f, 1.0f);
  const uint16_t norm = clampToU16((fraction + 1.0f) * 65535.0f / 2.0f);
  const uint8_t cmd[] = {kWahooSetSimGrade, (uint8_t)(norm & 0xFF), (uint8_t)(norm >> 8)};
  if (!write(cmd, sizeof(cmd), "setSimGrade")) return false;
  recordDebug([&](Kickr::Debug& d) { d.grade = (norm * 2.0f / 65535.0f - 1.0f) * 100.0f; });
  return true;
}

bool setWheelCircumference(float mm) {
  const uint16_t tenths = clampToU16(mm * 10.0f);
  const uint8_t cmd[] = {kWahooSetWheelCircum, (uint8_t)(tenths & 0xFF), (uint8_t)(tenths >> 8)};
  if (!write(cmd, sizeof(cmd), "setWheelCircumference")) return false;
  recordDebug([&](Kickr::Debug& d) { d.wheelMm = tenths / 10.0f; });
  return true;
}

bool setErgMode(uint16_t watts) {
  const uint8_t cmd[] = {kWahooSetErgMode, (uint8_t)(watts & 0xFF), (uint8_t)(watts >> 8)};
  if (!write(cmd, sizeof(cmd), "setErgMode")) return false;
  recordDebug([&](Kickr::Debug& d) {
    d.ergW = watts;
    d.mode = kWahooSetErgMode;
  });
  return true;
}

void disconnect() { hangUp(); }

Debug debug() {
  portENTER_CRITICAL(&s_dbgLock);
  const Debug copy = s_dbg;
  portEXIT_CRITICAL(&s_dbgLock);
  return copy;
}

bool acknowledged(uint8_t op) {
  const Exchange* exchange = exchangeFor(op);
  if (exchange == nullptr) return false;
  portENTER_CRITICAL(&s_dbgLock);
  const Exchange e = *exchange;
  portEXIT_CRITICAL(&s_dbgLock);
  return e.sent != 0 && e.answered == e.sent && (op == kWahooUnlock || e.status == 0x01);
}

}  // namespace Kickr
