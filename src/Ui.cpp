#include "Ui.h"

#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include "Config.h"
#include "DeviceRegistry.h"
#include "KickrClient.h"
#include "State.h"
#include "TrainerServer.h"

namespace {

// The CYD's touch controller hangs off its own SPI bus, separate from the
// panel's HSPI.
constexpr uint8_t kTouchSck = 25;
constexpr uint8_t kTouchMiso = 39;
constexpr uint8_t kTouchMosi = 32;
constexpr uint8_t kTouchCs = 33;

// Raw XPT2046 span. If the buttons feel offset, watch the serial log for
// "[touch] raw" lines while tapping the corners and adjust these.
constexpr int kTouchMinX = 200;
constexpr int kTouchMaxX = 3700;
constexpr int kTouchMinY = 240;
constexpr int kTouchMaxY = 3800;

constexpr int16_t kScreenW = 320;
constexpr int16_t kScreenH = 240;

// Ride screen: four status chips, then the profile button in the corner.
constexpr int16_t kChipY = 3;
constexpr int16_t kChipH = 20;
constexpr int16_t kChipW = 65;
constexpr int16_t kChipX[4] = {4, 73, 142, 211};
constexpr int16_t kProfileButtonX = 280;
constexpr int16_t kProfileButtonW = 36;

constexpr int16_t kRightX = 176;
constexpr int16_t kRightW = 140;
constexpr int16_t kCadenceButtonY = 98;
constexpr int16_t kCadenceButtonH = 20;

// Gear-count button, top of the left column.
constexpr int16_t kGearCountX = 10;
constexpr int16_t kGearCountY = 28;
constexpr int16_t kGearCountW = 150;
constexpr int16_t kGearCountH = 20;

// DEBUG button, right of the POWER label. Ends above y 46, where the power
// figure's padding starts painting.
constexpr int16_t kDebugButtonX = 248;
constexpr int16_t kDebugButtonY = 27;
constexpr int16_t kDebugButtonW = 68;
constexpr int16_t kDebugButtonH = 18;

// Debug screen: a title row holding the way back, then fixed lines of Font 2.
constexpr int16_t kBackButtonX = 248;
constexpr int16_t kBackButtonY = 2;
constexpr int16_t kBackButtonW = 68;
constexpr int16_t kBackButtonH = 18;
constexpr int16_t kDebugTop = 22;
constexpr int16_t kDebugLineH = 16;
constexpr int16_t kDebugValuePad = kScreenW - 8;
constexpr int16_t kDebugRightPad = 200;
constexpr uint32_t kDebugRefreshMs = 250;

// Profile screen: one row per field - label, value, unit, then -/+.
constexpr int16_t kProfileTop = 30;
constexpr int16_t kProfileRowPitch = 44;
constexpr int16_t kStepButtonW = 52;
constexpr int16_t kStepButtonH = 34;
constexpr int16_t kMinusX = 196;
constexpr int16_t kPlusX = 262;
constexpr int16_t kProfileValueRight = 164;
constexpr uint32_t kStepRepeatMs = 90;
constexpr uint32_t kStepFastAfterMs = 2500;  // then ten steps per repeat
constexpr int kStepRegionBase = 50;          // two touch regions per field

// Bottom button strip, shared by both screens.
constexpr int16_t kButtonTop = 198;
constexpr int16_t kButtonH = 38;

// Devices screen list.
constexpr int16_t kListTop = 30;
constexpr int16_t kRowH = 32;
constexpr int kVisibleRows = 5;

constexpr uint16_t kBg = TFT_BLACK;
constexpr uint16_t kDim = 0x4208;    // dark grey
constexpr uint16_t kLabel = 0x9CD3;  // light grey
constexpr uint16_t kGood = 0x0560;   // green
constexpr uint16_t kBad = 0x6000;    // dark red
constexpr uint16_t kAmber = 0x9A80;  // fallback warning
constexpr uint16_t kSavedRow = 0x0140;
constexpr uint16_t kRoleTrainer = 0xC300;  // orange
constexpr uint16_t kRoleShifter = 0x0560;  // green
constexpr uint16_t kRoleCadence = 0x001F;  // blue
constexpr uint16_t kRoleHeart = 0x4800;    // muted red: recognised, not usable

TFT_eSPI tft;
SPIClass touchSpi(VSPI);
XPT2046_Touchscreen touch(kTouchCs);

struct Cache {
  int gear = -1;
  int gearCount = -1;
  float ratio = 999.0f;
  int16_t power = -1;
  uint8_t cadence = 255;
  float appGrade = 999.0f;
  float appliedGrade = 999.0f;
  bool erg = false;
  uint16_t targetPower = 0xFFFF;
  bool kickr = false;
  bool di2 = false;
  bool app = false;
  bool sensor = false;
  int cadenceButton = -1;  // 0 trainer, 1 sensor, 2 sensor-but-fallen-back
  bool first = true;
};

Cache s_cache;

Ui::Screen s_screen = Ui::Screen::Devices;
int s_scroll = 0;
uint32_t s_lastListDrawMs = 0;

uint32_t s_touchHeldSinceMs = 0;
uint32_t s_lastRepeatMs = 0;
int s_heldRegion = 0;

// Device rows act on release so a tap and a hold can mean different things:
// tapping the same row twice used to connect and then immediately forget,
// which is exactly what someone does when the first tap looks like it did
// nothing.
int s_rowPress = -1;
uint32_t s_rowPressAtMs = 0;
bool s_rowPressFired = false;
constexpr uint32_t kForgetHoldMs = 1200;

// The devices screen's RIDE button sits exactly on the ride screen's HARDER+
// region, so without this the finger that changed screens is read as a fresh
// press on the new one ten milliseconds later.
bool s_ignoreUntilRelease = false;
uint32_t s_ignoreSetAtMs = 0;
uint32_t s_lastTouchLogMs = 0;

// A stuck-true touched() would otherwise latch s_ignoreUntilRelease forever and
// kill the screen, so the latch expires on its own too.
constexpr uint32_t kIgnoreTimeoutMs = 1000;

// ---------------------------------------------------------------------------
// Shared
// ---------------------------------------------------------------------------

void drawButton(int16_t x, int16_t w, const char* text, uint16_t colour, uint8_t font) {
  tft.fillRoundRect(x, kButtonTop, w, kButtonH, 6, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(text, x + w / 2, kButtonTop + kButtonH / 2, font);
}

// ---------------------------------------------------------------------------
// Ride screen
// ---------------------------------------------------------------------------

void drawChip(int index, const char* label, bool on) {
  const uint16_t colour = on ? kGood : kBad;
  const int16_t x = kChipX[index];
  tft.fillRoundRect(x, kChipY, kChipW, kChipH, 4, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(label, x + kChipW / 2, kChipY + kChipH / 2, 2);
}

int cadenceButtonState() {
  if (g_cfg.cadenceSource == CadenceSource::Trainer) return 0;
  return useSensorCadence() ? 1 : 2;
}

void drawCadenceButton(int state) {
  const uint16_t colour = (state == 0) ? kDim : (state == 1) ? TFT_NAVY : kAmber;
  const char* text = (state == 0) ? "CAD: TRAINER" : (state == 1) ? "CAD: SENSOR" : "CAD: NO SENSOR";
  tft.fillRoundRect(kRightX, kCadenceButtonY, kRightW, kCadenceButtonH, 4, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(text, kRightX + kRightW / 2, kCadenceButtonY + kCadenceButtonH / 2, 2);
}

void drawRideChrome() {
  tft.fillScreen(kBg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(kLabel, kBg);
  tft.drawString("POWER", kRightX, 30, 2);
  tft.drawString("GRADE", kRightX, 150, 2);

  tft.fillRoundRect(kDebugButtonX, kDebugButtonY, kDebugButtonW, kDebugButtonH, 4, kDim);
  tft.setTextColor(TFT_WHITE, kDim);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("DEBUG", kDebugButtonX + kDebugButtonW / 2, kDebugButtonY + kDebugButtonH / 2, 2);

  // A head and shoulders: the usual profile symbol, in a chip-sized button.
  tft.fillRoundRect(kProfileButtonX, kChipY, kProfileButtonW, kChipH, 4, kDim);
  const int16_t cx = kProfileButtonX + kProfileButtonW / 2;
  tft.fillCircle(cx, kChipY + 7, 4, TFT_WHITE);
  tft.fillRoundRect(cx - 8, kChipY + 13, 16, 7, 3, TFT_WHITE);
  tft.fillRect(cx - 8, kChipY + 17, 16, 3, TFT_WHITE);  // square off the bottom edge

  drawButton(4, 150, "- EASIER", kDim, 4);
  drawButton(166, 150, "HARDER +", kDim, 4);
}

// Nearest real Ultegra 50/34 x 11-34 combination, so a cassette gear reads as
// something a rider recognises. Display only - the ratio itself is what counts.
void nearestCombo(float ratio, int& ring, int& cog) {
  static const int kRings[] = {50, 34};
  static const int kCogs[] = {11, 12, 13, 14, 15, 17, 19, 21, 24, 27, 30, 34};
  float best = 1e9f;
  for (int r : kRings) {
    for (int c : kCogs) {
      const float d = fabsf(logf(((float)r / (float)c) / ratio));
      if (d < best) { best = d; ring = r; cog = c; }
    }
  }
}

// "34x14 2.43" on the real cassette; just "2.43" in the 32-gear mode, whose
// ratios are not a drivetrain's and would only match one approximately.
void formatRatio(char* buf, size_t size, float ratio) {
  if (g_cfg.gearCount != kGearCountLow) {
    snprintf(buf, size, "%.2f", ratio);
    return;
  }
  int ring = 0;
  int cog = 0;
  nearestCombo(ratio, ring, cog);
  snprintf(buf, size, "%dx%d %.2f", ring, cog, ratio);
}

void updateRide() {
  const bool first = s_cache.first;
  s_cache.first = false;

  if (first || s_cache.kickr != g_ride.kickrConnected) {
    s_cache.kickr = g_ride.kickrConnected;
    drawChip(0, "KICKR", s_cache.kickr);
  }
  if (first || s_cache.di2 != g_ride.di2Connected) {
    s_cache.di2 = g_ride.di2Connected;
    drawChip(1, "DI2", s_cache.di2);
  }
  if (first || s_cache.sensor != g_ride.sensorConnected) {
    s_cache.sensor = g_ride.sensorConnected;
    drawChip(2, "CAD", s_cache.sensor);
  }
  if (first || s_cache.app != g_ride.appConnected) {
    s_cache.app = g_ride.appConnected;
    drawChip(3, "APP", s_cache.app);
  }

  if (first || s_cache.gear != g_cfg.gear || s_cache.gearCount != g_cfg.gearCount) {
    s_cache.gear = g_cfg.gear;
    s_cache.gearCount = g_cfg.gearCount;

    tft.fillRoundRect(kGearCountX, kGearCountY, kGearCountW, kGearCountH, 4, kDim);
    tft.setTextColor(TFT_WHITE, kDim);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(0);
    tft.drawString(String(g_cfg.gearCount) + " GEARS", kGearCountX + kGearCountW / 2,
                   kGearCountY + kGearCountH / 2, 2);

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, kBg);
    tft.setTextSize(2);
    tft.setTextPadding(140);
    tft.drawString(String(g_cfg.gear), 10, 52, 7);
    tft.setTextSize(1);
  }

  const float ratio = gearRatio(g_cfg.gear);
  if (first || fabsf(ratio - s_cache.ratio) > 0.0005f) {
    s_cache.ratio = ratio;
    char buf[16];
    formatRatio(buf, sizeof(buf), ratio);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_CYAN, kBg);
    tft.setTextPadding(150);
    tft.drawString(buf, 10, 152, 4);
  }

  if (first || s_cache.power != g_ride.power) {
    s_cache.power = g_ride.power;
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, kBg);
    tft.setTextPadding(kRightW);
    tft.drawString(String(g_ride.power), kRightX, 46, 6);
  }

  const int cadenceButton = cadenceButtonState();
  if (first || cadenceButton != s_cache.cadenceButton) {
    s_cache.cadenceButton = cadenceButton;
    drawCadenceButton(cadenceButton);
  }

  const uint8_t cadence = effectiveCadence();
  if (first || s_cache.cadence != cadence) {
    s_cache.cadence = cadence;
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, kBg);
    tft.setTextPadding(kRightW);
    tft.drawString(String(cadence), kRightX, 120, 4);
  }

  const bool erg = g_ride.ergMode;
  const uint16_t target = g_ride.targetPower;
  if (first || fabsf(g_ride.appGrade - s_cache.appGrade) > 0.005f ||
      fabsf(g_ride.appliedGrade - s_cache.appliedGrade) > 0.005f || erg != s_cache.erg ||
      target != s_cache.targetPower) {
    s_cache.appGrade = g_ride.appGrade;
    s_cache.appliedGrade = g_ride.appliedGrade;
    s_cache.erg = erg;
    s_cache.targetPower = target;

    char buf[28];
    if (erg) {
      snprintf(buf, sizeof(buf), "ERG %uW", (unsigned)target);
    } else {
      snprintf(buf, sizeof(buf), "%.1f%%", g_ride.appliedGrade);
    }
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_YELLOW, kBg);
    tft.setTextPadding(kRightW);
    tft.drawString(buf, kRightX, 166, 4);
  }

  tft.setTextPadding(0);
}

// ---------------------------------------------------------------------------
// Debug screen
// ---------------------------------------------------------------------------

enum DebugSlot {
  kSlotGear,
  kSlotKickrHead,
  kSlotKickrValues,
  kSlotKickrCounters,
  kSlotAppHead,
  kSlotAppRaw,
  kSlotAppSim,
  kSlotAppErg,
  kSlotAppCounts,
  kSlotSentHead,
  kSlotSentValues,
  kSlotSentSim,
  kSlotSentAcks,
  kSlotToApp,
  kDebugSlots
};

struct DebugText {
  char text[64];
  uint16_t colour;
  bool valid;
};

DebugText s_debugText[kDebugSlots];
uint32_t s_lastDebugDrawMs = 0;
uint32_t s_rateAtMs = 0;
uint32_t s_ratePackets = 0;
uint32_t s_rateWrites = 0;
float s_kickrRate = 0.0f;
float s_appRate = 0.0f;

int16_t debugY(int row) { return kDebugTop + row * kDebugLineH; }

// Records the slot's new content; true if it differs from what is on screen.
bool debugChanged(int slot, const char* text, uint16_t colour) {
  DebugText& cached = s_debugText[slot];
  if (cached.valid && cached.colour == colour && strcmp(cached.text, text) == 0) return false;
  strlcpy(cached.text, text, sizeof(cached.text));
  cached.colour = colour;
  cached.valid = true;
  return true;
}

// Only repaints on a change. The padding wipes whatever a previous, longer
// string left behind, so there is no clear-then-draw flicker.
void debugLine(int slot, int16_t x, int16_t y, uint8_t datum, int16_t padding, const char* text,
               uint16_t colour) {
  if (!debugChanged(slot, text, colour)) return;
  tft.setTextDatum(datum);
  tft.setTextColor(colour, kBg);
  tft.setTextPadding(padding);
  tft.drawString(text, x, y, 2);
}

void debugValues(int slot, int row, const char* text, uint16_t colour) {
  debugLine(slot, 4, debugY(row), TL_DATUM, kDebugValuePad, text, colour);
}

void debugHeadInfo(int slot, int row, const char* text, uint16_t colour) {
  debugLine(slot, kScreenW - 4, debugY(row), TR_DATUM, kDebugRightPad, text, colour);
}

void formatAge(char* buf, size_t size, uint32_t stampMs) {
  const uint32_t age = elapsedSince(stampMs);
  if (age < 10000) {
    snprintf(buf, size, "%lu ms", (unsigned long)age);
  } else {
    snprintf(buf, size, "%lu s", (unsigned long)(age / 1000));
  }
}

void appendCount(char* buf, size_t size, const char* name, uint16_t count) {
  if (count == 0) return;
  const size_t used = strlen(buf);
  snprintf(buf + used, size - used, "  %s %u", name, (unsigned)count);
}

void updateRates(uint32_t now, uint32_t packets, uint32_t writes) {
  if (s_rateAtMs != 0 && now - s_rateAtMs < 1000) return;
  if (s_rateAtMs != 0) {
    const float seconds = (float)(now - s_rateAtMs) / 1000.0f;
    // Both counters restart on a reconnect; going backwards is that, not a rate.
    s_kickrRate = packets >= s_ratePackets ? (float)(packets - s_ratePackets) / seconds : 0.0f;
    s_appRate = writes >= s_rateWrites ? (float)(writes - s_rateWrites) / seconds : 0.0f;
  }
  s_rateAtMs = now;
  s_ratePackets = packets;
  s_rateWrites = writes;
}

uint16_t ackColour(uint8_t status, bool informational) {
  if (status == 0) return kDim;
  if (status == 0x01) return TFT_GREEN;
  return informational ? kLabel : TFT_RED;
}

// One colour per status, so a refusal stands out from the four that worked.
void drawAcks(int row, const Kickr::Debug& k) {
  char key[16];
  snprintf(key, sizeof(key), "%02x%02x%02x%02x%02x", k.ackUnlock, k.ackSim, k.ackGrade,
           k.ackWheel, k.ackErg);
  if (!debugChanged(kSlotSentAcks, key, 0)) return;

  struct Item {
    const char* name;
    uint8_t status;
    bool informational;
  };
  // The v5 answers unlock with 02 and works regardless, so that one is shown
  // but never flagged as a refusal.
  const Item items[] = {
      {"UNL", k.ackUnlock, true}, {"SIM", k.ackSim, false},   {"GRD", k.ackGrade, false},
      {"WHL", k.ackWheel, false}, {"ERG", k.ackErg, false},
  };

  const int16_t y = debugY(row);
  tft.fillRect(0, y, kScreenW, kDebugLineH, kBg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(kLabel, kBg);
  int16_t x = 4 + tft.drawString("ack", 4, y, 2) + 8;
  for (const Item& item : items) {
    tft.setTextColor(kLabel, kBg);
    x += tft.drawString(item.name, x, y, 2) + 4;
    char status[4];
    if (item.status == 0) {
      strlcpy(status, "--", sizeof(status));
    } else {
      snprintf(status, sizeof(status), "%02x", item.status);
    }
    tft.setTextColor(ackColour(item.status, item.informational), kBg);
    x += tft.drawString(status, x, y, 2) + 10;
  }
}

void drawDebugHeader(int row, const char* label, uint16_t colour) {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(colour, kBg);
  tft.setTextPadding(0);
  tft.drawString(label, 4, debugY(row), 2);
}

void drawDebugChrome() {
  tft.fillScreen(kBg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(kLabel, kBg);
  tft.setTextPadding(0);
  tft.drawString("DEBUG", 4, 3, 2);

  tft.fillRoundRect(kBackButtonX, kBackButtonY, kBackButtonW, kBackButtonH, 4, kDim);
  tft.setTextColor(TFT_WHITE, kDim);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("< RIDE", kBackButtonX + kBackButtonW / 2, kBackButtonY + kBackButtonH / 2, 2);

  drawDebugHeader(0, "KICKR > CYD", TFT_CYAN);
  drawDebugHeader(3, "APP > CYD", TFT_ORANGE);
  drawDebugHeader(8, "CYD > KICKR", TFT_GREENYELLOW);
  drawDebugHeader(12, "CYD > APP", TFT_VIOLET);

  memset(s_debugText, 0, sizeof(s_debugText));
  s_lastDebugDrawMs = 0;
  s_rateAtMs = 0;
  s_kickrRate = 0.0f;
  s_appRate = 0.0f;
}

void updateDebug() {
  const uint32_t now = millis();
  if (s_lastDebugDrawMs != 0 && now - s_lastDebugDrawMs < kDebugRefreshMs) return;
  s_lastDebugDrawMs = now;

  const Kickr::Debug k = Kickr::debug();
  const AppLink::Debug a = AppLink::debug();
  const AppLink::SimParams sim = AppLink::simParams();
  updateRates(now, k.packets, a.writes);

  char buf[64];
  char age[16];

  // Title row: the gear, so the wheel size sent below can be read against it.
  char ratioText[16];
  formatRatio(ratioText, sizeof(ratioText), gearRatio(g_cfg.gear));
  snprintf(buf, sizeof(buf), "gear %d/%d  %s", g_cfg.gear, g_cfg.gearCount, ratioText);
  debugLine(kSlotGear, 60, 3, TL_DATUM, kBackButtonX - 64, buf, TFT_WHITE);

  // --- KICKR > CYD: the Cycling Power Measurement, as received.
  if (!g_ride.kickrConnected) {
    debugHeadInfo(kSlotKickrHead, 0, "no link", TFT_RED);
  } else if (k.packets == 0) {
    debugHeadInfo(kSlotKickrHead, 0, "waiting", kLabel);
  } else {
    formatAge(age, sizeof(age), k.lastPacketMs);
    snprintf(buf, sizeof(buf), "flags %04x  %.1f/s  %s", k.flags, s_kickrRate, age);
    // The KICKR goes quiet when the flywheel stops, so this is only alarming
    // mid-effort - but that is exactly when it needs to be seen.
    debugHeadInfo(kSlotKickrHead, 0, buf,
                  elapsedSince(k.lastPacketMs) > 2000 ? TFT_RED : kLabel);
  }
  snprintf(buf, sizeof(buf), "%d W   %.1f km/h   %u rpm   trq %u", k.power,
           (float)g_ride.speedKph, (unsigned)g_ride.trainerCadence, (unsigned)k.torque);
  debugValues(kSlotKickrValues, 1, buf, TFT_WHITE);
  snprintf(buf, sizeof(buf), "wheel %lu @%u   crank %u @%u", (unsigned long)k.wheelRevs,
           (unsigned)k.wheelTime, (unsigned)k.crankRevs, (unsigned)k.crankTime);
  debugValues(kSlotKickrCounters, 2, buf, TFT_WHITE);

  // --- APP > CYD: FTMS control point writes.
  if (!g_ride.appConnected) {
    debugHeadInfo(kSlotAppHead, 3, "no app", TFT_RED);
  } else if (a.writes == 0) {
    debugHeadInfo(kSlotAppHead, 3, "no writes yet", kLabel);
  } else {
    formatAge(age, sizeof(age), a.lastWriteMs);
    snprintf(buf, sizeof(buf), "%.1f/s  %s", s_appRate, age);
    debugHeadInfo(kSlotAppHead, 3, buf, kLabel);
  }

  if (a.writes == 0) {
    strlcpy(buf, "rx -", sizeof(buf));
  } else {
    size_t used = strlcpy(buf, "rx", sizeof(buf));
    const size_t shown = min<size_t>(a.lastLen, sizeof(a.last));
    for (size_t b = 0; b < shown && used < sizeof(buf); b++) {
      used += snprintf(buf + used, sizeof(buf) - used, " %02x", a.last[b]);
    }
    if (a.lastLen > sizeof(a.last) && used < sizeof(buf)) {
      snprintf(buf + used, sizeof(buf) - used, " ..");
    }
  }
  debugValues(kSlotAppRaw, 4, buf, TFT_WHITE);

  if (a.simParams == 0) {
    debugValues(kSlotAppSim, 5, "no sim params yet", kDim);
  } else {
    snprintf(buf, sizeof(buf), "grade %+.2f%%  crr .%04u  cw %.2f  wind %.1f",
             (float)g_ride.appGrade, (unsigned)lroundf(sim.crr * 10000.0f), sim.cw, sim.windMps);
    debugValues(kSlotAppSim, 5, buf, TFT_WHITE);
  }

  if (g_ride.ergMode) {
    snprintf(buf, sizeof(buf), "ERG ON   target %u W", (unsigned)g_ride.targetPower);
    debugValues(kSlotAppErg, 6, buf, TFT_YELLOW);
  } else {
    debugValues(kSlotAppErg, 6, "ERG off", kLabel);
  }

  strlcpy(buf, "cmds", sizeof(buf));
  appendCount(buf, sizeof(buf), "sim", a.simParams);
  appendCount(buf, sizeof(buf), "erg", a.targetPower);
  appendCount(buf, sizeof(buf), "res", a.targetResistance);
  appendCount(buf, sizeof(buf), "ctrl", a.requestControl);
  appendCount(buf, sizeof(buf), "start", a.start);
  appendCount(buf, sizeof(buf), "stop", a.stop);
  appendCount(buf, sizeof(buf), "reset", a.reset);
  appendCount(buf, sizeof(buf), "other", a.other);
  debugValues(kSlotAppCounts, 7, buf, a.writes == 0 ? kDim : TFT_WHITE);

  // --- CYD > KICKR: the last value of each command, and the trainer's answer.
  if (!g_ride.kickrConnected) {
    debugHeadInfo(kSlotSentHead, 8, "no link", TFT_RED);
  } else if (k.mode == kWahooSetErgMode) {
    debugHeadInfo(kSlotSentHead, 8, "ERG mode", TFT_YELLOW);
  } else if (k.mode == kWahooSetSimMode) {
    debugHeadInfo(kSlotSentHead, 8, "SIM mode", TFT_WHITE);
  } else {
    debugHeadInfo(kSlotSentHead, 8, "starting", kLabel);
  }

  char grade[16] = "-";
  char wheel[16] = "-";
  char erg[16] = "-";
  if (!isnan(k.grade)) snprintf(grade, sizeof(grade), "%+.2f%%", k.grade);
  if (!isnan(k.wheelMm)) snprintf(wheel, sizeof(wheel), "%.0f mm", k.wheelMm);
  if (k.ergW >= 0) snprintf(erg, sizeof(erg), "%ld W", (long)k.ergW);
  snprintf(buf, sizeof(buf), "grade %s  wheel %s  erg %s", grade, wheel, erg);
  debugValues(kSlotSentValues, 9, buf, TFT_WHITE);

  if (isnan(k.simKg)) {
    debugValues(kSlotSentSim, 10, "sim -", kDim);
  } else {
    snprintf(buf, sizeof(buf), "sim %.1f kg  crr %.3f  cw %.3f", k.simKg, k.simCrr, k.simCw);
    debugValues(kSlotSentSim, 10, buf, TFT_WHITE);
  }

  drawAcks(11, k);

  // --- CYD > APP: Indoor Bike Data, and the power packet relayed alongside it.
  if (!g_ride.appConnected) {
    debugLine(kSlotToApp, 78, debugY(12), TL_DATUM, kScreenW - 82, "no app", TFT_RED);
  } else {
    snprintf(buf, sizeof(buf), "%d W  %u rpm %s  %.1f kph", (int)g_ride.power,
             (unsigned)effectiveCadence(), useSensorCadence() ? "sensor" : "trainer",
             (float)g_ride.speedKph);
    debugLine(kSlotToApp, 78, debugY(12), TL_DATUM, kScreenW - 82, buf, TFT_WHITE);
  }

  tft.setTextPadding(0);
}

// ---------------------------------------------------------------------------
// Profile screen
// ---------------------------------------------------------------------------

struct ProfileRow {
  const char* label;
  const char* unit;
  uint8_t decimals;
};
const ProfileRow kProfileRows[kProfileFieldCount] = {
    {"Rider weight", "kg", 1},
    {"Bike weight", "kg", 1},
    {"Wheel circ.", "mm", 0},
    {"Rider height", "cm", 0},
};

Profile s_profileDraft;
float s_profileShown[kProfileFieldCount];
bool s_profileShownValid = false;

int16_t profileRowY(int row) { return kProfileTop + row * kProfileRowPitch; }

void drawStepButton(int16_t x, int16_t y, const char* text) {
  tft.fillRoundRect(x, y, kStepButtonW, kStepButtonH, 6, kDim);
  tft.setTextColor(TFT_WHITE, kDim);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(text, x + kStepButtonW / 2, y + kStepButtonH / 2, 4);
}

void drawProfileChrome() {
  tft.fillScreen(kBg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(kLabel, kBg);
  tft.setTextPadding(0);
  tft.drawString("PROFILE", 4, 3, 2);

  tft.fillRoundRect(kBackButtonX, kBackButtonY, kBackButtonW, kBackButtonH, 4, kDim);
  tft.setTextColor(TFT_WHITE, kDim);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("< RIDE", kBackButtonX + kBackButtonW / 2, kBackButtonY + kBackButtonH / 2, 2);

  for (int row = 0; row < kProfileFieldCount; row++) {
    const int16_t y = profileRowY(row);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(kLabel, kBg);
    tft.drawString(kProfileRows[row].label, 8, y + 9, 2);
    tft.drawString(kProfileRows[row].unit, kProfileValueRight + 4, y + 9, 2);
    drawStepButton(kMinusX, y, "-");
    drawStepButton(kPlusX, y, "+");
  }

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(kDim, kBg);
  tft.drawString("Saved on the way back. Hold to go faster.", 8, 212, 2);

  s_profileShownValid = false;
}

void updateProfile() {
  for (int row = 0; row < kProfileFieldCount; row++) {
    const float value = s_profileDraft.value[row];
    if (s_profileShownValid && fabsf(value - s_profileShown[row]) < 1e-3f) continue;
    s_profileShown[row] = value;

    char buf[12];
    snprintf(buf, sizeof(buf), "%.*f", (int)kProfileRows[row].decimals, value);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_WHITE, kBg);
    tft.setTextPadding(70);
    tft.drawString(buf, kProfileValueRight, profileRowY(row) + 4, 4);
  }
  s_profileShownValid = true;
  tft.setTextPadding(0);
}

// Region ids for the -/+ pair of each row: base + 2*row (minus), +1 (plus).
void stepFromRegion(int region, uint32_t heldMs) {
  const int offset = region - kStepRegionBase;
  const int direction = (offset % 2) ? 1 : -1;
  const int steps = heldMs > kStepFastAfterMs ? 10 : 1;
  stepProfile(s_profileDraft, (ProfileField)(offset / 2), direction * steps);
}

// ---------------------------------------------------------------------------
// Devices screen
// ---------------------------------------------------------------------------

uint16_t roleColour(Registry::Role role) {
  switch (role) {
    case Registry::Role::Trainer: return kRoleTrainer;
    case Registry::Role::Shifter: return kRoleShifter;
    case Registry::Role::Cadence: return kRoleCadence;
    case Registry::Role::HeartRate: return kRoleHeart;
    default: return kDim;
  }
}

const SavedDevice* savedSlotFor(const Registry::Entry& entry) {
  if (g_cfg.trainer.matches(entry.addr)) return &g_cfg.trainer;
  if (g_cfg.shifter.matches(entry.addr)) return &g_cfg.shifter;
  if (g_cfg.cadence.matches(entry.addr)) return &g_cfg.cadence;
  return nullptr;
}

void drawDevicesChrome() {
  tft.fillScreen(kBg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(kLabel, kBg);
  tft.drawString("TAP CONNECT   HOLD FORGET", 6, 6, 2);

  drawButton(4, 72, "UP", kDim, 4);
  drawButton(82, 72, "DOWN", kDim, 4);
  drawButton(160, 156, "RIDE >", kGood, 4);
}

void drawDeviceList() {
  const size_t total = Registry::count();

  const int maxScroll = (int)total > kVisibleRows ? (int)total - kVisibleRows : 0;
  if (s_scroll > maxScroll) s_scroll = maxScroll;
  if (s_scroll < 0) s_scroll = 0;

  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(kLabel, kBg);
  tft.setTextPadding(120);
  char header[32];
  snprintf(header, sizeof(header), "%u found", (unsigned)total);
  tft.drawString(header, kScreenW - 6, 6, 2);

  for (int row = 0; row < kVisibleRows; row++) {
    const int16_t y = kListTop + row * kRowH;
    Registry::Entry entry;
    const bool present = Registry::get((size_t)(s_scroll + row), entry);

    if (!present) {
      tft.fillRect(0, y, kScreenW, kRowH - 2, kBg);
      continue;
    }

    const SavedDevice* saved = savedSlotFor(entry);
    const uint16_t rowBg = saved != nullptr ? kSavedRow : kBg;
    tft.fillRect(0, y, kScreenW, kRowH - 2, rowBg);

    const uint16_t tag = roleColour(entry.role);
    tft.fillRoundRect(4, y + 3, 58, kRowH - 8, 3, tag);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, tag);
    tft.setTextPadding(0);
    tft.drawString(Registry::roleName(entry.role), 33, y + kRowH / 2 - 1, 2);

    const bool stale = Registry::ageMs(entry) > Registry::kStaleAfterMs;
    const uint16_t nameColour =
        stale ? kDim : (saved != nullptr ? TFT_GREENYELLOW : TFT_WHITE);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(nameColour, rowBg);
    tft.setTextPadding(200);
    tft.drawString(entry.name, 68, y + 7, 2);

    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(kLabel, rowBg);
    tft.setTextPadding(40);
    tft.drawString(String(entry.rssi), kScreenW - 6, y + 7, 2);
  }

  tft.setTextPadding(0);
}

void updateDevices() {
  const uint32_t now = millis();
  if (s_cache.first) {
    s_cache.first = false;
    s_lastListDrawMs = 0;
  }
  // The list is small and changes on its own as the scan runs, so a periodic
  // repaint is simpler than tracking every field for a diff.
  if (now - s_lastListDrawMs < 600) return;
  s_lastListDrawMs = now;
  drawDeviceList();
}

}  // namespace

namespace Ui {

void begin() {
  tft.init();
  // Landscape, upside down (3 rather than 1) to suit how the board is mounted.
  tft.setRotation(3);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

  touchSpi.begin(kTouchSck, kTouchMiso, kTouchMosi, kTouchCs);
  touch.begin(touchSpi);
  // Touch stays at rotation 1 so the raw calibration above still applies; the
  // 180-degree flip is done by reversing the ranges in pollTouch().
  touch.setRotation(1);
}

Screen screen() { return s_screen; }

void setScreen(Screen next) {
  s_screen = next;
  s_cache = Cache();
  s_heldRegion = 0;
  s_ignoreUntilRelease = true;
  s_ignoreSetAtMs = millis();
  if (next == Screen::Ride) {
    drawRideChrome();
  } else if (next == Screen::Debug) {
    drawDebugChrome();
  } else if (next == Screen::Profile) {
    s_profileDraft = currentProfile();
    drawProfileChrome();
  } else {
    s_scroll = 0;
    drawDevicesChrome();
  }
  update();
}

void update() {
  if (s_screen == Screen::Ride) {
    updateRide();
  } else if (s_screen == Screen::Debug) {
    updateDebug();
  } else if (s_screen == Screen::Profile) {
    updateProfile();
  } else {
    updateDevices();
  }
}

const Profile& profileDraft() { return s_profileDraft; }

Touch pollTouch() {
  Touch result;
  const uint32_t now = millis();

  if (!touch.touched()) {
    if (s_rowPress >= 0 && !s_rowPressFired) {
      result.action = Action::SelectDevice;
      result.index = s_rowPress;
    }
    s_rowPress = -1;
    s_rowPressFired = false;
    s_heldRegion = 0;
    s_ignoreUntilRelease = false;
    return result;
  }
  if (s_ignoreUntilRelease && (now - s_ignoreSetAtMs) < kIgnoreTimeoutMs) return result;
  s_ignoreUntilRelease = false;

  const TS_Point p = touch.getPoint();
  const int16_t x = (int16_t)map(p.x, kTouchMaxX, kTouchMinX, 0, kScreenW);
  const int16_t y = (int16_t)map(p.y, kTouchMaxY, kTouchMinY, 0, kScreenH);

  // Unconditional and rate limited. This used to print only once a tap landed
  // inside a region, which is precisely the case that cannot happen when the
  // calibration is wrong - so the one thing needed to fix it was never logged.
  if (now - s_lastTouchLogMs > 250) {
    s_lastTouchLogMs = now;
    Serial.printf("[touch] raw %d,%d z=%d -> %d,%d\n", p.x, p.y, p.z, x, y);
  }

  // Region ids are arbitrary but must be stable, so a held finger is not read
  // as a fresh press on every poll.
  int region = 0;
  Action action = Action::None;
  int index = -1;

  if (s_screen == Screen::Ride) {
    // Ahead of the chip row, whose tap zone reaches a couple of pixels into
    // this button.
    if (x >= kDebugButtonX - 8 && y >= kDebugButtonY - 2 &&
        y <= kDebugButtonY + kDebugButtonH + 6) {
      region = 15;
      action = Action::OpenDebug;
    } else if (y < kChipY + kChipH + 4 && x >= kProfileButtonX - 3) {
      region = 16;
      action = Action::OpenProfile;
    } else if (y < kChipY + kChipH + 4) {
      region = 10;
      action = Action::OpenDevices;
    } else if (y >= kCadenceButtonY - 6 && y <= kCadenceButtonY + kCadenceButtonH + 6 &&
               x >= kRightX - 6) {
      region = 11;
      action = Action::ToggleCadenceSource;
    } else if (y <= kGearCountY + kGearCountH + 6 && x <= kGearCountX + kGearCountW + 6) {
      region = 14;
      action = Action::ToggleGearCount;
    } else if (y >= kButtonTop - 10) {
      region = (x < kScreenW / 2) ? 12 : 13;
      action = (region == 12) ? Action::ShiftDown : Action::ShiftUp;
    }
  } else if (s_screen == Screen::Debug) {
    if (x >= kBackButtonX - 8 && y <= kBackButtonY + kBackButtonH + 8) {
      region = 30;
      action = Action::StartRide;
    }
  } else if (s_screen == Screen::Profile) {
    if (x >= kBackButtonX - 8 && y <= kBackButtonY + kBackButtonH + 6) {
      region = 40;
      action = Action::CloseProfile;
    } else if (y >= kProfileTop - 4) {
      const int row = (y - kProfileTop + 4) / kProfileRowPitch;
      const int16_t rowY = profileRowY(row);
      if (row < kProfileFieldCount && y <= rowY + kStepButtonH + 4) {
        if (x >= kMinusX - 6 && x < kMinusX + kStepButtonW + 6) {
          region = kStepRegionBase + 2 * row;
        } else if (x >= kPlusX - 6) {
          region = kStepRegionBase + 2 * row + 1;
        }
      }
    }
  } else {
    if (y >= kListTop && y < kListTop + kVisibleRows * kRowH) {
      const int row = (y - kListTop) / kRowH;
      index = s_scroll + row;
      region = 100 + row;
      action = Action::None;  // decided on release, or by holding
    } else if (y >= kButtonTop - 10) {
      if (x < 78) {
        region = 20;
        action = Action::ScrollUp;
      } else if (x < 156) {
        region = 21;
        action = Action::ScrollDown;
      } else {
        region = 22;
        action = Action::StartRide;
      }
    }
  }

  if (region == 0) {
    s_heldRegion = 0;
    s_rowPress = -1;
    return result;
  }

  if (region >= 100) {
    if (s_heldRegion != region) {
      s_heldRegion = region;
      s_rowPress = index;
      s_rowPressAtMs = now;
      s_rowPressFired = false;
      Serial.printf("[touch] row %d pressed\n", index);
    } else if (!s_rowPressFired && (now - s_rowPressAtMs) > kForgetHoldMs) {
      s_rowPressFired = true;
      result.action = Action::ForgetDevice;
      result.index = s_rowPress;
    }
    return result;
  }

  const bool isShift = (region == 12 || region == 13);
  const bool isStep =
      region >= kStepRegionBase && region < kStepRegionBase + 2 * kProfileFieldCount;

  if (s_heldRegion != region) {
    Serial.printf("[touch] region %d\n", region);
    s_heldRegion = region;
    s_touchHeldSinceMs = now;
    s_lastRepeatMs = now;
    if (isStep) {
      stepFromRegion(region, 0);
      return result;
    }
    result.action = action;
    result.index = index;
    if (action == Action::ScrollUp) s_scroll--;
    if (action == Action::ScrollDown) s_scroll++;
    if (action == Action::ScrollUp || action == Action::ScrollDown) s_lastListDrawMs = 0;
    return result;
  }

  // Auto-repeat on shifting and profile steps only. Repeating a device
  // selection or a screen change is never what a resting thumb meant.
  const uint32_t heldMs = now - s_touchHeldSinceMs;
  if (isShift && heldMs > 600 && now - s_lastRepeatMs > 180) {
    s_lastRepeatMs = now;
    result.action = action;
  } else if (isStep && heldMs > 500 && now - s_lastRepeatMs > kStepRepeatMs) {
    s_lastRepeatMs = now;
    stepFromRegion(region, heldMs);
  }
  return result;
}

}  // namespace Ui
