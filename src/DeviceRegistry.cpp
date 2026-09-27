#include "DeviceRegistry.h"

#include "Config.h"
#include "State.h"

namespace {

Registry::Entry s_entries[Registry::kMaxEntries];
size_t s_count = 0;
portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

bool sameAddress(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

bool isSaved(const uint8_t addr[6]) {
  return g_cfg.trainer.matches(addr) || g_cfg.shifter.matches(addr) ||
         g_cfg.cadence.matches(addr);
}

bool containsIgnoreCase(const std::string& haystack, const char* needle) {
  std::string a = haystack, b = needle;
  for (auto& c : a) c = (char)toupper((unsigned char)c);
  for (auto& c : b) c = (char)toupper((unsigned char)c);
  return a.find(b) != std::string::npos;
}

bool startsWithIgnoreCase(const std::string& value, const char* prefix) {
  const size_t n = strlen(prefix);
  if (value.size() < n) return false;
  for (size_t i = 0; i < n; i++) {
    if (toupper((unsigned char)value[i]) != toupper((unsigned char)prefix[i])) return false;
  }
  return true;
}

// Best guess from the advertisement alone. Anything left Unknown can still be
// assigned by probing it - the roles are confirmed by which service the device
// actually exposes once connected.
Registry::Role classify(const NimBLEAdvertisedDevice* device, const std::string& name) {
  if (device->isAdvertisingService(NimBLEUUID(UUID_SVC_SHIMANO)) ||
      device->isAdvertisingService(NimBLEUUID(UUID_SVC_SHIMANO_ALT)) ||
      startsWithIgnoreCase(name, kDi2NamePrefix)) {
    return Registry::Role::Shifter;
  }
  if (device->isAdvertisingService(NimBLEUUID(UUID_SVC_CSC)) ||
      startsWithIgnoreCase(name, kSensorNamePrefix)) {
    return Registry::Role::Cadence;
  }
  if (device->isAdvertisingService(NimBLEUUID(UUID_SVC_HEART_RATE))) {
    return Registry::Role::HeartRate;
  }
  if (containsIgnoreCase(name, kKickrNamePrefix) ||
      device->isAdvertisingService(NimBLEUUID(UUID_SVC_CYCLING_POWER))) {
    return Registry::Role::Trainer;
  }
  return Registry::Role::Unknown;
}

}  // namespace

namespace Registry {

void note(const NimBLEAdvertisedDevice* device) {
  const std::string name = device->getName();
  const Role role = classify(device, name);

  // Nameless beacons with nothing recognisable would just bury the list.
  if (name.empty() && role == Role::Unknown) return;

  Entry candidate;
  memcpy(candidate.addr, device->getAddress().getVal(), 6);
  candidate.addrType = device->getAddress().getType();
  candidate.rssi = device->getRSSI();
  candidate.lastSeenMs = millis();
  candidate.role = role;
  if (name.empty()) {
    snprintf(candidate.name, sizeof(candidate.name), "%s", device->getAddress().toString().c_str());
  } else {
    snprintf(candidate.name, sizeof(candidate.name), "%s", name.c_str());
  }

  portENTER_CRITICAL(&s_lock);
  for (size_t i = 0; i < s_count; i++) {
    if (sameAddress(s_entries[i].addr, candidate.addr)) {
      s_entries[i].rssi = candidate.rssi;
      s_entries[i].lastSeenMs = candidate.lastSeenMs;
      if (candidate.role != Role::Unknown) s_entries[i].role = candidate.role;
      if (!name.empty()) memcpy(s_entries[i].name, candidate.name, sizeof(candidate.name));
      portEXIT_CRITICAL(&s_lock);
      return;
    }
  }
  if (s_count < kMaxEntries) {
    s_entries[s_count++] = candidate;
  } else {
    // A full table used to mean the list silently stopped growing, which locks
    // out a trainer that only starts advertising once the flywheel is spun.
    // Evict the least recently seen row, never one the rider has chosen.
    size_t victim = kMaxEntries;
    uint32_t oldest = 0;
    for (size_t i = 0; i < s_count; i++) {
      if (isSaved(s_entries[i].addr)) continue;
      const uint32_t age = candidate.lastSeenMs - s_entries[i].lastSeenMs;
      if (victim == kMaxEntries || age > oldest) {
        oldest = age;
        victim = i;
      }
    }
    if (victim < kMaxEntries) s_entries[victim] = candidate;
  }
  portEXIT_CRITICAL(&s_lock);
}

size_t count() {
  portENTER_CRITICAL(&s_lock);
  const size_t n = s_count;
  portEXIT_CRITICAL(&s_lock);
  return n;
}

bool get(size_t index, Entry& out) {
  portENTER_CRITICAL(&s_lock);
  const bool ok = index < s_count;
  if (ok) out = s_entries[index];
  portEXIT_CRITICAL(&s_lock);
  return ok;
}

bool seenRecently(const uint8_t addr[6], uint32_t withinMs, Entry& out) {
  bool found = false;
  portENTER_CRITICAL(&s_lock);
  for (size_t i = 0; i < s_count; i++) {
    if (sameAddress(s_entries[i].addr, addr) && elapsedSince(s_entries[i].lastSeenMs) < withinMs) {
      out = s_entries[i];
      found = true;
      break;
    }
  }
  portEXIT_CRITICAL(&s_lock);
  return found;
}

// getVal() hands back NimBLE's internal little-endian bytes, but the
// NimBLEAddress(uint8_t[6], type) constructor reverse-copies because it expects
// them the way they are written down. Round-tripping through it therefore
// byte-reverses the address, and every connect targets a device that does not
// exist. Go through ble_addr_t, which is native order at both ends.
NimBLEAddress addressOf(const Entry& entry) {
  ble_addr_t raw;
  raw.type = entry.addrType;
  memcpy(raw.val, entry.addr, sizeof(raw.val));
  return NimBLEAddress(raw);
}

const char* roleName(Role role) {
  switch (role) {
    case Role::Trainer: return "TRAIN";
    case Role::Shifter: return "SHIFT";
    case Role::Cadence: return "CAD";
    case Role::HeartRate: return "HR";
    default: return "?";
  }
}

}  // namespace Registry
