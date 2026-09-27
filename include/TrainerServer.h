#pragma once
#include <Arduino.h>

namespace AppLink {

struct SimParams {
  float windMps = 0.0f;
  float crr = 0.0f;
  float cw = 0.0f;
};

void begin();

// Restart advertising if it is not running and no app is connected. Cheap to
// call; the bridge going unfindable mid-ride is worse than a redundant check.
void ensureAdvertising();

// Forward the KICKR's Cycling Power Measurement packet untouched.
void relayCyclingPower(const uint8_t* data, size_t len);

// Push one FTMS Indoor Bike Data notification from the current ride state.
void publishIndoorBikeData();

// Flush any pending FTMS control point response. Call from a task, not from
// a BLE callback - indicating from inside onWrite can deadlock the host.
void flushControlPointResponse();

// Takes the current parameters and clears the changed flag in one step, so a
// write landing mid-read cannot have its flag consumed while the stale values
// are what get sent. Returns true if they had changed since the last take.
bool takeSimParams(SimParams& out);

SimParams simParams();

// What the app has been writing to the FTMS control point, for the debug
// screen. Cleared when an app connects.
struct Debug {
  uint8_t  last[8] = {0};  // leading bytes of the latest write
  uint8_t  lastLen = 0;    // its full length, which can exceed sizeof(last)
  uint32_t writes = 0;
  uint32_t lastWriteMs = 0;

  // Writes per opcode.
  uint16_t requestControl = 0;    // 0x00
  uint16_t reset = 0;             // 0x01
  uint16_t targetResistance = 0;  // 0x04
  uint16_t targetPower = 0;       // 0x05
  uint16_t start = 0;             // 0x07
  uint16_t stop = 0;              // 0x08
  uint16_t simParams = 0;         // 0x11
  uint16_t other = 0;
};

Debug debug();

}  // namespace AppLink
