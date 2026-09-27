#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

#include "Config.h"

namespace Registry {

// HeartRate is recognised only so a strap is not mistaken for something we
// can use; it is listed and never probed.
enum class Role : uint8_t { Unknown = 0, Trainer, Shifter, Cadence, HeartRate };

struct Entry {
  uint8_t addr[6] = {0};
  uint8_t addrType = 0;
  char name[22] = {0};
  int8_t rssi = 0;
  uint32_t lastSeenMs = 0;
  Role role = Role::Unknown;
};

constexpr size_t kMaxEntries = 12;

// Called from the scan callback. Updates an existing row or appends a new one.
void note(const NimBLEAdvertisedDevice* device);

size_t count();
bool get(size_t index, Entry& out);

// A saved device is only worth a connect attempt while it is actually
// advertising - otherwise the call sits on the connect timeout doing nothing.
bool seenRecently(const uint8_t addr[6], uint32_t withinMs, Entry& out);

NimBLEAddress addressOf(const Entry& entry);

// Milliseconds since this row last advertised.
inline uint32_t ageMs(const Entry& entry) { return elapsedSince(entry.lastSeenMs); }

// A row older than this has most likely gone to sleep; connecting to it can
// only sit on the timeout. Generous, because an idle trainer can leave long
// gaps between advertisements and a tight threshold makes the list flicker.
static constexpr uint32_t kStaleAfterMs = 30000;
const char* roleName(Role role);

}  // namespace Registry
