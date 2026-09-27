#include "CadenceSensor.h"

#include "BleLink.h"
#include "Config.h"
#include "State.h"

namespace {

NimBLEClient* s_client = nullptr;
DisconnectGate s_gate;

uint16_t s_lastRev = 0;
uint16_t s_lastTime = 0;
bool s_primed = false;

void reset() {
  s_primed = false;
  g_ride.sensorCadence = 0;
  g_ride.sensorCrankValid = false;
}

// Cycling Speed and Cadence measurement:
//   flags u8, [wheel: u32 revs + u16 event time], [crank: u16 revs + u16 time]
// Crank event time ticks at 1/1024 s, the same unit the power service uses,
// so the counters can be handed to the app untouched.
void onMeasurement(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  (void)chr;
  (void)isNotify;
  if (len < 1) return;

  const uint8_t flags = data[0];
  size_t i = 1;
  if (flags & 0x01) i += 6;  // wheel revolution data - unused, speed is modelled by the app
  if (!(flags & 0x02)) return;
  if (len < i + 4) return;

  const uint16_t rev = (uint16_t)(data[i] | (data[i + 1] << 8));
  const uint16_t eventTime = (uint16_t)(data[i + 2] | (data[i + 3] << 8));
  const uint32_t now = millis();

  if (s_primed) {
    const uint16_t dRev = (uint16_t)(rev - s_lastRev);
    const uint16_t dT = (uint16_t)(eventTime - s_lastTime);
    // Stopping is noticed by expireStoppedCadence(), not here: a repeated
    // packet only says nothing moved since the last one.
    if (dRev > 0 && dT > 0) {
      const float rpm = (float)dRev * 60.0f * 1024.0f / (float)dT;
      if (rpm < 250.0f) g_ride.sensorCadence = (uint8_t)lroundf(rpm);
      g_ride.sensorRevMs = now;
    }
  } else {
    s_primed = true;
  }

  s_lastRev = rev;
  s_lastTime = eventTime;
  g_ride.sensorCrankRev = rev;
  g_ride.sensorCrankTime = eventTime;
  g_ride.sensorCrankValid = true;
  g_ride.lastSensorDataMs = now;
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    (void)client;
    Serial.println("[sensor] connected");
  }
  void onDisconnect(NimBLEClient* client, int reason) override {
    (void)client;
    Serial.printf("[sensor] disconnected, reason %d\n", reason);
    s_gate.signal();
    g_ride.sensorConnected = false;
    reset();
  }
};

ClientCallbacks s_callbacks;

// Hangs up and waits for the link to be fully torn down, so the next role
// probe is not rejected against a connection that is still closing.
void hangUp() {
  if (s_client == nullptr || !s_client->isConnected()) return;
  s_gate.arm();
  s_client->disconnect();
  s_gate.wait();
}

}  // namespace

namespace Sensor {

bool connected() { return s_client != nullptr && s_client->isConnected() && g_ride.sensorConnected; }

bool connectTo(const NimBLEAddress& addr) {
  if (s_client == nullptr) {
    s_client = NimBLEDevice::createClient();
    s_client->setClientCallbacks(&s_callbacks, false);
    s_client->setConnectTimeout(8000);
  }
  if (!s_client->connect(addr)) {
    Serial.println("[sensor] connect failed");
    return false;
  }

  NimBLERemoteService* csc = s_client->getService(NimBLEUUID(UUID_SVC_CSC));
  if (csc == nullptr) {
    Serial.println("[sensor] no speed and cadence service");
    hangUp();
    return false;
  }

  NimBLERemoteCharacteristic* chr = csc->getCharacteristic(NimBLEUUID(UUID_CHR_CSC_MEASUREMENT));
  if (chr == nullptr || !chr->canNotify()) {
    Serial.println("[sensor] no measurement notifications");
    hangUp();
    return false;
  }
  chr->subscribe(true, onMeasurement);

  reset();
  g_ride.sensorConnected = true;
  return true;
}

void dropIfStale() {
  if (s_client != nullptr && !s_client->isConnected()) g_ride.sensorConnected = false;
}

void disconnect() { hangUp(); }

}  // namespace Sensor
