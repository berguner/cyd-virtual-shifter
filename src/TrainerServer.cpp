#include "TrainerServer.h"

#include <NimBLEDevice.h>

#include "Config.h"
#include "State.h"

namespace {

NimBLEServer* s_server = nullptr;
NimBLECharacteristic* s_cpMeasurement = nullptr;
NimBLECharacteristic* s_indoorBikeData = nullptr;
NimBLECharacteristic* s_controlPoint = nullptr;
NimBLECharacteristic* s_ftmsStatus = nullptr;

AppLink::SimParams s_sim;
bool s_simDirty = true;

// FTMS wants one Response Code per write, and an app can land several writes
// between two passes of the loop, so a single slot silently dropped acks.
constexpr size_t kResponseQueueLen = 8;
uint8_t s_responses[kResponseQueueLen][3];
size_t s_respHead = 0;
size_t s_respTail = 0;
portMUX_TYPE s_respLock = portMUX_INITIALIZER_UNLOCKED;

// Fitness Machine Status notifications: how a real trainer tells the app its
// control state changed. Queued with the responses, for the same reason.
struct StatusMessage {
  uint8_t len;
  uint8_t data[8];
};
constexpr size_t kStatusQueueLen = 8;
StatusMessage s_status[kStatusQueueLen];
size_t s_statusHead = 0;
size_t s_statusTail = 0;
volatile uint16_t s_statusSubscription = 0;

// MyWhoosh sends the same sim-params write to switch ERG off as to switch it
// back on, and follows the switch-on with a target only if the target changed
// since it last sent one - it ignores status notifications saying the trainer
// left ERG. So a sim-params write that ends ERG only pauses it, and an
// identical one next resumes it at the kept target. A write with different
// parameters means the rider is riding the simulation, and ends the pause.
// NimBLE task only.
bool s_ergPaused = false;
uint32_t s_ergPausedAtMs = 0;
// MyWhoosh also repeats the write on its own, 2.5 s apart, with no toggle; a
// person switching ERG off and back on took 5 s or more.
constexpr uint32_t kErgResumeMinMs = 3000;
uint8_t s_lastSimParams[6] = {0};
bool s_haveLastSimParams = false;

// Simulation parameters are written from the NimBLE task and read from the
// loop, so the flag and the values have to be taken together or a write that
// lands in between has its flag eaten while the stale values get sent.
portMUX_TYPE s_simLock = portMUX_INITIALIZER_UNLOCKED;

// FTMS control point opcodes.
constexpr uint8_t kOpRequestControl = 0x00;
constexpr uint8_t kOpReset = 0x01;
constexpr uint8_t kOpSetTargetResistance = 0x04;
constexpr uint8_t kOpSetTargetPower = 0x05;
constexpr uint8_t kOpStartOrResume = 0x07;
constexpr uint8_t kOpStopOrPause = 0x08;
constexpr uint8_t kOpSetSimParams = 0x11;

constexpr uint8_t kResultSuccess = 0x01;
constexpr uint8_t kResultNotSupported = 0x02;

// Written from the NimBLE task, read by the UI.
AppLink::Debug s_dbg;
portMUX_TYPE s_dbgLock = portMUX_INITIALIZER_UNLOCKED;

uint16_t& opCounter(AppLink::Debug& d, uint8_t opcode) {
  switch (opcode) {
    case kOpRequestControl: return d.requestControl;
    case kOpReset: return d.reset;
    case kOpSetTargetResistance: return d.targetResistance;
    case kOpSetTargetPower: return d.targetPower;
    case kOpStartOrResume: return d.start;
    case kOpStopOrPause: return d.stop;
    case kOpSetSimParams: return d.simParams;
    default: return d.other;
  }
}

void recordWrite(const uint8_t* data, size_t len) {
  // Every write, in order: whether an app follows a target-power write with
  // sim parameters (which cancels ERG) is only visible as a sequence.
  char hex[64];
  size_t n = 0;
  for (size_t b = 0; b < len && n + 4 < sizeof(hex); b++) {
    n += snprintf(hex + n, sizeof(hex) - n, "%02x ", data[b]);
  }
  Serial.printf("[app] rx %s(status sub %u)\n", hex, (unsigned)s_statusSubscription);

  portENTER_CRITICAL(&s_dbgLock);
  memcpy(s_dbg.last, data, min(len, sizeof(s_dbg.last)));
  s_dbg.lastLen = (uint8_t)min<size_t>(len, 255);
  s_dbg.writes++;
  s_dbg.lastWriteMs = millis();
  opCounter(s_dbg, data[0])++;
  portEXIT_CRITICAL(&s_dbgLock);
}

// Leaving ERG latched means gears stop doing anything and the rider stays
// pinned at the last workout wattage, so every path out of a workout has to
// clear it - not just the app volunteering a new simulation grade.
void leaveErgMode() {
  g_ride.ergMode = false;
  g_ride.targetPower = 0;
  s_ergPaused = false;
}

void queueResponse(uint8_t opcode, uint8_t result) {
  portENTER_CRITICAL(&s_respLock);
  const size_t next = (s_respHead + 1) % kResponseQueueLen;
  if (next != s_respTail) {
    s_responses[s_respHead][0] = 0x80;
    s_responses[s_respHead][1] = opcode;
    s_responses[s_respHead][2] = result;
    s_respHead = next;
  }
  portEXIT_CRITICAL(&s_respLock);
}

// FTMS Fitness Machine Status op codes.
constexpr uint8_t kStatusReset = 0x01;
constexpr uint8_t kStatusStopped = 0x02;
constexpr uint8_t kStatusStarted = 0x04;
constexpr uint8_t kStatusTargetResistance = 0x07;
constexpr uint8_t kStatusTargetPower = 0x08;
constexpr uint8_t kStatusSimParams = 0x12;

// The parameters are the write's own, echoed back as the new state.
void queueStatus(uint8_t op, const uint8_t* params, size_t paramLen) {
  portENTER_CRITICAL(&s_respLock);
  const size_t next = (s_statusHead + 1) % kStatusQueueLen;
  if (next != s_statusTail) {
    StatusMessage& msg = s_status[s_statusHead];
    const size_t n = min(paramLen, sizeof(msg.data) - 1);
    msg.data[0] = op;
    memcpy(msg.data + 1, params, n);
    msg.len = (uint8_t)(n + 1);
    s_statusHead = next;
  }
  portEXIT_CRITICAL(&s_respLock);
}

class StatusCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* chr, NimBLEConnInfo& info, uint16_t subValue) override {
    (void)chr;
    (void)info;
    s_statusSubscription = subValue;
    Serial.printf("[app] machine status subscription %u\n", (unsigned)subValue);
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    (void)info;
    (void)server;
    Serial.println("[app] connected");
    g_ride.appConnected = true;
    s_haveLastSimParams = false;
    s_ergPaused = false;
    portENTER_CRITICAL(&s_dbgLock);
    s_dbg = AppLink::Debug();
    portEXIT_CRITICAL(&s_dbgLock);
    // Deliberately not advertising while an app is connected. Doing so holds a
    // connection slot open and means the advertising state has to be juggled
    // across connect and disconnect, which is how the bridge ended up
    // invisible after a mid-ride drop.
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    (void)info;
    Serial.printf("[app] disconnected, reason %d\n", reason);
    g_ride.appConnected = server->getConnectedCount() > 0;
    if (!g_ride.appConnected) {
      leaveErgMode();
      s_statusSubscription = 0;
    }
    // advertiseOnDisconnect handles the restart; ensureAdvertising is the
    // backstop if it ever does not.
  }
};

class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& info) override {
    (void)info;
    const NimBLEAttValue value = chr->getValue();
    if (value.length() == 0) return;

    const uint8_t* d = value.data();
    const size_t len = value.length();
    const uint8_t opcode = d[0];
    recordWrite(d, len);

    switch (opcode) {
      case kOpRequestControl:
        queueResponse(opcode, kResultSuccess);
        break;

      case kOpStartOrResume:
        queueResponse(opcode, kResultSuccess);
        queueStatus(kStatusStarted, nullptr, 0);
        break;

      case kOpReset:
        leaveErgMode();
        queueResponse(opcode, kResultSuccess);
        queueStatus(kStatusReset, nullptr, 0);
        break;

      case kOpStopOrPause: {
        leaveErgMode();
        queueResponse(opcode, kResultSuccess);
        const uint8_t stopOrPause = len >= 2 ? d[1] : 0x01;
        queueStatus(kStatusStopped, &stopOrPause, 1);
        break;
      }

      case kOpSetTargetPower:
        if (len >= 3) {
          const int16_t watts = (int16_t)(d[1] | (d[2] << 8));
          g_ride.targetPower = (uint16_t)max<int16_t>(0, watts);
          g_ride.ergMode = true;
          s_ergPaused = false;
          queueResponse(opcode, kResultSuccess);
          queueStatus(kStatusTargetPower, d + 1, 2);
        } else {
          queueResponse(opcode, kResultNotSupported);
        }
        break;

      case kOpSetTargetResistance:
        // Accepted so apps that probe it do not mark us uncontrollable, but
        // gears own the resistance here, so there is nothing to apply.
        queueResponse(opcode, kResultSuccess);
        if (len >= 2) queueStatus(kStatusTargetResistance, d + 1, 1);
        break;

      case kOpSetSimParams:
        if (len >= 7) {
          const int16_t wind = (int16_t)(d[1] | (d[2] << 8));
          const int16_t grade = (int16_t)(d[3] | (d[4] << 8));
          const float crr = (float)d[5] / 10000.0f;
          const float cw = (float)d[6] / 100.0f;

          g_ride.appGrade = (float)grade / 100.0f;

          const bool repeat = s_haveLastSimParams && memcmp(s_lastSimParams, d + 1, 6) == 0;
          memcpy(s_lastSimParams, d + 1, 6);
          s_haveLastSimParams = true;
          if (g_ride.ergMode) {
            g_ride.ergMode = false;
            s_ergPaused = g_ride.targetPower > 0;
            s_ergPausedAtMs = millis();
          } else if (s_ergPaused && repeat) {
            // Too soon to be a person: leave the pause armed for the real one.
            if (elapsedSince(s_ergPausedAtMs) >= kErgResumeMinMs) {
              g_ride.ergMode = true;
              s_ergPaused = false;
              Serial.printf("[app] repeated sim params while ERG paused - resuming ERG at %u W\n",
                            (unsigned)g_ride.targetPower);
            }
          } else {
            s_ergPaused = false;
          }

          const float windMps = (float)wind / 1000.0f;
          portENTER_CRITICAL(&s_simLock);
          if (fabsf(crr - s_sim.crr) > 1e-6f || fabsf(cw - s_sim.cw) > 1e-6f ||
              fabsf(windMps - s_sim.windMps) > 1e-4f) {
            s_sim.crr = crr;
            s_sim.cw = cw;
            s_sim.windMps = windMps;
            s_simDirty = true;
          }
          portEXIT_CRITICAL(&s_simLock);
          queueResponse(opcode, kResultSuccess);
          queueStatus(kStatusSimParams, d + 1, 6);
        } else {
          queueResponse(opcode, kResultNotSupported);
        }
        break;

      default:
        queueResponse(opcode, kResultNotSupported);
        break;
    }
  }
};

// Crank counters as the app sees them. Rebuilt rather than copied so that two
// things the raw stream gets wrong can be fixed on the way through.
struct CrankStream {
  bool primed = false;
  bool fromSensor = false;
  uint16_t srcRev = 0;      // source counters at its last revolution
  uint16_t srcTime = 0;
  uint32_t srcRevMs = 0;    // when that revolution was seen
  uint16_t rev = 0;         // what the app has been sent
  uint16_t time = 0;
  uint16_t timeAtRev = 0;   // sent event time of the last real revolution
};
CrankStream s_crank;

void normaliseCrank(uint16_t srcRev, uint16_t srcTime, bool fromSensor, uint16_t& outRev,
                    uint16_t& outTime) {
  const uint32_t now = millis();
  const bool first = !s_crank.primed;
  if (first) {
    s_crank.primed = true;
    s_crank.rev = srcRev;
    s_crank.time = srcTime;
  }
  if (first || fromSensor != s_crank.fromSensor) {
    // Switching between trainer and sensor would otherwise hand the app two
    // unrelated counter streams back to back - a burst of nonsense cadence.
    // Rebase onto the new source without moving what the app has seen.
    s_crank.fromSensor = fromSensor;
    s_crank.srcRev = srcRev;
    s_crank.srcTime = srcTime;
    s_crank.srcRevMs = now;
    s_crank.timeAtRev = s_crank.time;
  } else {
    const uint16_t dRev = (uint16_t)(srcRev - s_crank.srcRev);
    const uint16_t dT = (uint16_t)(srcTime - s_crank.srcTime);
    if (dRev != 0) {
      uint16_t t = (uint16_t)(s_crank.timeAtRev + dT);
      // Must land after anything already sent, or the app reads a zero or
      // negative interval. Only reachable after a very long stop.
      if ((int16_t)(t - s_crank.time) <= 0) t = (uint16_t)(s_crank.time + dT);
      s_crank.rev = (uint16_t)(s_crank.rev + dRev);
      s_crank.time = s_crank.timeAtRev = t;
      s_crank.srcRev = srcRev;
      s_crank.srcTime = srcTime;
      s_crank.srcRevMs = now;
    } else if (effectiveCadence() == 0) {
      // Stopped, on the same rule the display and the FTMS cadence use. The
      // source keeps repeating its last event time, which apps
      // read as "no news" and hold the old cadence for several seconds. Moving
      // the time on with no new revolution reads as zero straight away. Capped
      // short of the 64 s wrap; by then the app is already showing zero.
      const uint32_t ticks = min<uint32_t>((now - s_crank.srcRevMs) * 1024 / 1000, 60000);
      s_crank.time = (uint16_t)(s_crank.timeAtRev + ticks);
    }
  }
  outRev = s_crank.rev;
  outTime = s_crank.time;
}

ServerCallbacks s_serverCallbacks;
ControlPointCallbacks s_controlPointCallbacks;
StatusCallbacks s_statusCallbacks;

void addDeviceInformation() {
  NimBLEService* dis = s_server->createService(NimBLEUUID(UUID_SVC_DEVICE_INFO));
  dis->createCharacteristic((uint16_t)0x2A29, NIMBLE_PROPERTY::READ)->setValue("CYD");
  dis->createCharacteristic((uint16_t)0x2A24, NIMBLE_PROPERTY::READ)->setValue("Virtual Shifter");
  dis->createCharacteristic((uint16_t)0x2A26, NIMBLE_PROPERTY::READ)->setValue("1.0.0");
}

void addCyclingPower() {
  NimBLEService* cps = s_server->createService(NimBLEUUID(UUID_SVC_CYCLING_POWER));

  s_cpMeasurement = cps->createCharacteristic(NimBLEUUID(UUID_CHR_CP_MEASUREMENT), NIMBLE_PROPERTY::NOTIFY);

  // Wheel (bit 2) and crank (bit 3) revolution data supported.
  const uint8_t feature[4] = {0x0C, 0x00, 0x00, 0x00};
  cps->createCharacteristic(NimBLEUUID(UUID_CHR_CP_FEATURE), NIMBLE_PROPERTY::READ)->setValue(feature, 4);

  const uint8_t location = 13;  // rear hub
  cps->createCharacteristic(NimBLEUUID(UUID_CHR_SENSOR_LOCATION), NIMBLE_PROPERTY::READ)->setValue(&location, 1);
}

void addFitnessMachine() {
  NimBLEService* ftms = s_server->createService(NimBLEUUID(UUID_SVC_FTMS));

  // Machine features: cadence (bit 1) + power measurement (bit 14).
  // Target features: resistance (bit 2), power (bit 3), sim parameters (bit 13).
  const uint8_t feature[8] = {0x02, 0x40, 0x00, 0x00, 0x0C, 0x20, 0x00, 0x00};
  ftms->createCharacteristic(NimBLEUUID(UUID_CHR_FTMS_FEATURE), NIMBLE_PROPERTY::READ)->setValue(feature, 8);

  s_indoorBikeData = ftms->createCharacteristic(NimBLEUUID(UUID_CHR_INDOOR_BIKE_DATA), NIMBLE_PROPERTY::NOTIFY);

  s_controlPoint = ftms->createCharacteristic(
      NimBLEUUID(UUID_CHR_FTMS_CONTROL_POINT), NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  s_controlPoint->setCallbacks(&s_controlPointCallbacks);

  s_ftmsStatus = ftms->createCharacteristic(NimBLEUUID(UUID_CHR_FTMS_STATUS), NIMBLE_PROPERTY::NOTIFY);
  s_ftmsStatus->setCallbacks(&s_statusCallbacks);

  const uint8_t resistanceRange[6] = {0x00, 0x00, 0xE8, 0x03, 0x0A, 0x00};  // 0 .. 100.0, step 1.0
  ftms->createCharacteristic(NimBLEUUID(UUID_CHR_RESISTANCE_RANGE), NIMBLE_PROPERTY::READ)
      ->setValue(resistanceRange, 6);

  const uint8_t powerRange[6] = {0x00, 0x00, 0xD0, 0x07, 0x01, 0x00};  // 0 .. 2000 W, step 1
  ftms->createCharacteristic(NimBLEUUID(UUID_CHR_POWER_RANGE), NIMBLE_PROPERTY::READ)->setValue(powerRange, 6);
}

void startAdvertising() {
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  // Before setName: the name is only diverted into scan data if scan response
  // is already enabled, and that is the escape hatch if the payload ever grows
  // past 31 bytes.
  adv->enableScanResponse(true);
  adv->setName(kServerName);
  adv->addServiceUUID(NimBLEUUID(UUID_SVC_FTMS));
  adv->addServiceUUID(NimBLEUUID(UUID_SVC_CYCLING_POWER));

  // FTMS advertising service data: machine available, type = indoor bike.
  const uint8_t serviceData[3] = {0x01, 0x20, 0x00};
  adv->setServiceData(NimBLEUUID(UUID_SVC_FTMS), serviceData, 3);

  adv->start();
}

}  // namespace

namespace AppLink {

void begin() {
  s_server = NimBLEDevice::createServer();
  s_server->setCallbacks(&s_serverCallbacks);
  s_server->advertiseOnDisconnect(true);

  addDeviceInformation();
  addCyclingPower();
  addFitnessMachine();
  startAdvertising();
}

void relayCyclingPower(const uint8_t* data, size_t len) {
  if (s_cpMeasurement == nullptr || !g_ride.appConnected) return;
  if (len < 4 || len > 28) {
    s_cpMeasurement->setValue(data, len);
    s_cpMeasurement->notify();
    return;
  }

  uint8_t buf[32];
  memcpy(buf, data, len);
  uint16_t flags = (uint16_t)(buf[0] | (buf[1] << 8));

  size_t i = 4;
  if (flags & 0x0001) i += 1;  // pedal power balance
  if (flags & 0x0004) i += 2;  // accumulated torque
  if (flags & 0x0010) i += 6;  // wheel revolution data

  // Crank counters come from the sensor when the rider picked it, otherwise
  // from the trainer's own field. Both count in 1/1024 s, so they carry over
  // as-is and the app derives cadence exactly as it would from the source.
  const bool hasCrank = (flags & 0x0020) && len >= i + 4;
  const bool fromSensor = useSensorCadence() && g_ride.sensorCrankValid;
  uint16_t srcRev = 0;
  uint16_t srcTime = 0;
  if (fromSensor) {
    srcRev = g_ride.sensorCrankRev;
    srcTime = g_ride.sensorCrankTime;
  } else if (hasCrank) {
    srcRev = (uint16_t)(buf[i] | (buf[i + 1] << 8));
    srcTime = (uint16_t)(buf[i + 2] | (buf[i + 3] << 8));
  } else {
    s_cpMeasurement->setValue(data, len);
    s_cpMeasurement->notify();
    return;
  }

  if (!hasCrank) {
    // Only append when nothing follows the crank slot, so the packet stays
    // well formed.
    if ((flags & 0xFFC0) != 0 || len != i) {
      s_cpMeasurement->setValue(data, len);
      s_cpMeasurement->notify();
      return;
    }
    flags |= 0x0020;
    buf[0] = (uint8_t)(flags & 0xFF);
    buf[1] = (uint8_t)(flags >> 8);
    len = i + 4;
  }

  uint16_t rev;
  uint16_t eventTime;
  normaliseCrank(srcRev, srcTime, fromSensor, rev, eventTime);
  buf[i + 0] = (uint8_t)(rev & 0xFF);
  buf[i + 1] = (uint8_t)(rev >> 8);
  buf[i + 2] = (uint8_t)(eventTime & 0xFF);
  buf[i + 3] = (uint8_t)(eventTime >> 8);
  s_cpMeasurement->setValue(buf, len);
  s_cpMeasurement->notify();
}

void publishIndoorBikeData() {
  if (s_indoorBikeData == nullptr || !g_ride.appConnected) return;

  // Bit 0 clear => instantaneous speed present; bit 2 cadence; bit 6 power.
  // The speed field is carried even though nothing reads it: this payload is
  // positional, and MyWhoosh read cadence as speed and power as cadence when
  // the field was omitted, leaving power at zero.
  const uint16_t flags = 0x0044;
  const float hundredthsKph = g_ride.speedKph * 100.0f;
  const uint16_t speed = (uint16_t)constrain(hundredthsKph, 0.0f, 65535.0f);
  const float halfRpm = (float)effectiveCadence() * 2.0f;
  const uint16_t cadence = (uint16_t)constrain(halfRpm, 0.0f, 65535.0f);
  const int16_t power = g_ride.power;

  const uint8_t payload[8] = {
      (uint8_t)(flags & 0xFF),   (uint8_t)(flags >> 8),
      (uint8_t)(speed & 0xFF),   (uint8_t)(speed >> 8),
      (uint8_t)(cadence & 0xFF), (uint8_t)(cadence >> 8),
      (uint8_t)(power & 0xFF),   (uint8_t)((power >> 8) & 0xFF),
  };
  s_indoorBikeData->setValue(payload, sizeof(payload));
  s_indoorBikeData->notify();
}

void flushControlPointResponse() {
  if (s_controlPoint == nullptr) return;

  uint8_t response[3];
  for (;;) {
    portENTER_CRITICAL(&s_respLock);
    const bool empty = s_respTail == s_respHead;
    if (!empty) {
      memcpy(response, s_responses[s_respTail], sizeof(response));
      s_respTail = (s_respTail + 1) % kResponseQueueLen;
    }
    portEXIT_CRITICAL(&s_respLock);
    if (empty) break;

    // Pass the bytes explicitly. NimBLECharacteristic::writeEvent overwrites
    // the stored value with the client's request before onWrite runs, and the
    // no-argument indicate() sends whatever is stored - which can be that
    // request echoed back as its own response.
    s_controlPoint->indicate(response, sizeof(response));
  }

  // After the responses: the spec has the status follow the procedure's
  // response, not precede it.
  if (s_ftmsStatus == nullptr) return;
  StatusMessage msg;
  for (;;) {
    portENTER_CRITICAL(&s_respLock);
    const bool empty = s_statusTail == s_statusHead;
    if (!empty) {
      msg = s_status[s_statusTail];
      s_statusTail = (s_statusTail + 1) % kStatusQueueLen;
    }
    portEXIT_CRITICAL(&s_respLock);
    if (empty) return;
    s_ftmsStatus->notify(msg.data, msg.len);
  }
}

void ensureAdvertising() {
  if (s_server == nullptr || g_ride.appConnected) return;
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  if (adv->isAdvertising()) return;
  Serial.println("[app] advertising had stopped - restarting");
  adv->start();
}

bool takeSimParams(SimParams& out) {
  portENTER_CRITICAL(&s_simLock);
  const bool dirty = s_simDirty;
  s_simDirty = false;
  out = s_sim;
  portEXIT_CRITICAL(&s_simLock);
  return dirty;
}

SimParams simParams() {
  portENTER_CRITICAL(&s_simLock);
  const SimParams copy = s_sim;
  portEXIT_CRITICAL(&s_simLock);
  return copy;
}

Debug debug() {
  portENTER_CRITICAL(&s_dbgLock);
  const Debug copy = s_dbg;
  portEXIT_CRITICAL(&s_dbgLock);
  return copy;
}

}  // namespace AppLink
