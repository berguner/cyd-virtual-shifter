#include "Di2Client.h"

#include "BleLink.h"
#include "Config.h"
#include "State.h"

namespace {

NimBLEClient* s_client = nullptr;
DisconnectGate s_gate;
Di2::Handler s_handler = nullptr;

// One entry per D-Fly channel. The derailleur re-sends the whole channel
// vector on every change, so we diff against what we saw last time.
uint8_t s_lastValue[kDflyChannels] = {0};
bool s_primed = false;

// A long press keeps arriving as a repeated short-press flag until release.
// Latch it so a held button fires once rather than machine-gunning.
bool s_held[kDflyChannels] = {false};

void onChannelData(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  (void)chr;
  (void)isNotify;
  if (len < 2) return;

  // data[0] is a header byte; one byte per channel follows. Always work on a
  // full-width vector - priming only the channels a short packet happened to
  // carry leaves the rest at zero, and the next full packet then reads as a
  // press nobody made.
  uint8_t current[kDflyChannels] = {0};
  const size_t provided = min((size_t)kDflyChannels, len - 1);
  for (size_t i = 0; i < provided; i++) current[i] = data[i + 1];

  if (!s_primed) {
    memcpy(s_lastValue, current, sizeof(s_lastValue));
    s_primed = true;
    return;
  }

  for (size_t i = 0; i < kDflyChannels; i++) {
    const uint8_t value = current[i];
    if (value == s_lastValue[i]) continue;
    s_lastValue[i] = value;

    const int channel = (int)i + 1;
    if (value & kDflyDoublePress) {
      s_held[i] = false;
      if (s_handler) s_handler(channel, Di2::Press::Double);
    } else if (value & kDflyLongPress) {
      if (!s_held[i]) {
        s_held[i] = true;
        if (s_handler) s_handler(channel, Di2::Press::Long);
      }
    } else if (value & kDflyShortPress) {
      // A long press reports short-press first; only fire if not already held.
      if (!s_held[i] && s_handler) s_handler(channel, Di2::Press::Short);
    } else {
      s_held[i] = false;
    }
  }
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    (void)client;
    Serial.println("[di2] connected");
  }
  void onDisconnect(NimBLEClient* client, int reason) override {
    (void)client;
    Serial.printf("[di2] disconnected, reason %d\n", reason);
    s_gate.signal();
    g_ride.di2Connected = false;
    s_primed = false;
  }
};

ClientCallbacks s_callbacks;

NimBLERemoteService* findShimanoService(NimBLEClient* client) {
  NimBLERemoteService* svc = client->getService(NimBLEUUID(UUID_SVC_SHIMANO));
  if (svc == nullptr) svc = client->getService(NimBLEUUID(UUID_SVC_SHIMANO_ALT));
  return svc;
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

namespace Di2 {

void setHandler(Handler handler) { s_handler = handler; }

bool connected() { return s_client != nullptr && s_client->isConnected() && g_ride.di2Connected; }

bool connectTo(const NimBLEAddress& addr) {
  if (s_client == nullptr) {
    s_client = NimBLEDevice::createClient();
    s_client->setClientCallbacks(&s_callbacks, false);
    s_client->setConnectTimeout(8000);
    s_client->setConnectionParams(12, 24, 0, 400);
  }
  if (!s_client->connect(addr)) {
    Serial.println("[di2] connect failed");
    return false;
  }

  NimBLERemoteService* svc = findShimanoService(s_client);
  if (svc == nullptr) {
    Serial.println("[di2] no Shimano BLE service");
    hangUp();
    return false;
  }

  NimBLERemoteCharacteristic* chr = svc->getCharacteristic(NimBLEUUID(UUID_CHR_DFLY_CHANNEL));
  if (chr == nullptr) {
    Serial.println("[di2] no D-Fly channel characteristic");
    hangUp();
    return false;
  }

  // Shimano indicates rather than notifies, but accept either.
  const bool ok = chr->canIndicate() ? chr->subscribe(false, onChannelData)
                                     : chr->subscribe(true, onChannelData);
  if (!ok) {
    Serial.println("[di2] subscribe failed");
    hangUp();
    return false;
  }

  s_primed = false;
  for (int i = 0; i < kDflyChannels; i++) s_held[i] = false;
  g_ride.di2Connected = true;
  return true;
}

void dropIfStale() {
  if (s_client != nullptr && !s_client->isConnected()) g_ride.di2Connected = false;
}

void disconnect() { hangUp(); }

}  // namespace Di2
