#pragma once
#include <Arduino.h>

// Age of a millis() timestamp, never negative.
//
// Timestamps here are written from BLE callbacks on one core and read from the
// loop on the other, so a stamp can sit a hair ahead of the reading task's idea
// of "now" - the loop samples millis() once at the top and a notification lands
// while it works. Plain unsigned subtraction turns that into ~4.29e9 ms, which
// sails past every timeout in this firmware. Signed arithmetic also keeps the
// 49-day millis() rollover honest.
inline uint32_t elapsedSince(uint32_t stamp) {
  const int32_t delta = (int32_t)(millis() - stamp);
  return delta > 0 ? (uint32_t)delta : 0;
}

// ---------------------------------------------------------------------------
// Who we pretend to be for Zwift / MyWhoosh.
// ---------------------------------------------------------------------------
static constexpr char kServerName[] = "CYDShift";

// ---------------------------------------------------------------------------
// Upstream: Wahoo KICKR v5 (2020).
//
// The v5 has no FTMS and no native Zwift virtual shifting - Wahoo ruled both
// out for v4/v5. It exposes the standard Cycling Power Service plus a Wahoo
// control point characteristic that lives inside it.
// ---------------------------------------------------------------------------
static constexpr char kKickrNamePrefix[] = "KICKR";
#define UUID_SVC_CYCLING_POWER      ((uint16_t)0x1818)
#define UUID_CHR_CP_MEASUREMENT     ((uint16_t)0x2A63)
#define UUID_CHR_CP_FEATURE         ((uint16_t)0x2A65)
#define UUID_CHR_SENSOR_LOCATION    ((uint16_t)0x2A5D)
#define UUID_CHR_WAHOO_CONTROL      "a026e005-0a7d-4ab3-97fa-f1500f9feb8b"

// Wahoo control point opcodes.
enum WahooOp : uint8_t {
  kWahooUnlock             = 0x20,
  kWahooSetResistanceMode  = 0x40,
  kWahooSetStandardMode    = 0x41,
  kWahooSetErgMode         = 0x42,
  kWahooSetSimMode         = 0x43,
  kWahooSetSimCRR          = 0x44,
  kWahooSetSimWindResist   = 0x45,
  kWahooSetSimGrade        = 0x46,
  kWahooSetSimWindSpeed    = 0x47,
  kWahooSetWheelCircum     = 0x48,
};

// ---------------------------------------------------------------------------
// Upstream: Shimano 12-speed Di2 (R8150). BLE lives in the rear derailleur,
// which advertises as "RDR....". Hood-top buttons must be assigned to D-Fly
// channels in E-TUBE PROJECT Cycling - otherwise nothing is broadcast here.
// ---------------------------------------------------------------------------
static constexpr char kDi2NamePrefix[] = "RDR";
#define UUID_SVC_SHIMANO      "000018ef-5348-494d-414e-4f5f424c4500"
#define UUID_SVC_SHIMANO_ALT  "000018ff-5348-494d-414e-4f5f424c4500"
#define UUID_CHR_DFLY_CHANNEL "00002ac2-5348-494d-414e-4f5f424c4500"

// Press flags in each per-channel byte of the D-Fly indication.
static constexpr uint8_t kDflyShortPress  = 0x10;
static constexpr uint8_t kDflyLongPress   = 0x20;
static constexpr uint8_t kDflyDoublePress = 0x40;
static constexpr int     kDflyChannels    = 4;

// ---------------------------------------------------------------------------
// Upstream, optional: a BLE cadence/speed sensor (COOSPO and friends) on the
// standard Cycling Speed and Cadence service. The KICKR v5 infers cadence from
// flywheel speed, so a real crank sensor is usually the better number.
// ---------------------------------------------------------------------------
#define UUID_SVC_CSC              ((uint16_t)0x1816)

// Recognised purely so a heart rate strap is tagged rather than probed - it is
// not relayed, pair it with the training app directly.
#define UUID_SVC_HEART_RATE       ((uint16_t)0x180D)
#define UUID_CHR_CSC_MEASUREMENT  ((uint16_t)0x2A5B)
static constexpr char kSensorNamePrefix[] = "COOSPO";

// The KICKR needs a moment between the unlock and being put into sim mode.
// qdomyos-zwift waits 700 ms on the same trainer family.
static constexpr uint32_t kUnlockSettleMs = 700;

// How long to wait for the trainer to answer a control command before
// resending it. Answers normally come within one connection interval.
static constexpr uint32_t kKickrAckTimeoutMs = 1500;

// The KICKR reports wheel revolutions against its own configured circumference,
// and its sim physics uses the same figure, so it is pushed to the trainer on
// connect. 700x30c; the profile screen overrides it.
static constexpr float kDefaultWheelMm = 2146.0f;

// How long the trainer can go quiet before its readings are treated as stale.
static constexpr uint32_t kTrainerSilenceMs = 8000;

// A selected sensor silent this long, while the trainer still sees pedalling,
// is treated as dropped out and the trainer's cadence is used instead.
static constexpr uint32_t kCadenceTimeoutMs = 4000;

// The trainer is only put into ERG once the flywheel is doing at least this -
// about 30 rpm on the real 34/17 - and taken out again once cadence has read
// zero this long. See applyTrainerState().
static constexpr float kErgMinEntryKph = 8.0f;
static constexpr uint32_t kErgStopGraceMs = 4000;

// ---------------------------------------------------------------------------
// Downstream: what the training app sees.
// ---------------------------------------------------------------------------
#define UUID_SVC_FTMS               ((uint16_t)0x1826)
#define UUID_CHR_FTMS_FEATURE       ((uint16_t)0x2ACC)
#define UUID_CHR_INDOOR_BIKE_DATA   ((uint16_t)0x2AD2)
#define UUID_CHR_RESISTANCE_RANGE   ((uint16_t)0x2AD6)
#define UUID_CHR_POWER_RANGE        ((uint16_t)0x2AD8)
#define UUID_CHR_FTMS_CONTROL_POINT ((uint16_t)0x2AD9)
#define UUID_CHR_FTMS_STATUS        ((uint16_t)0x2ADA)
#define UUID_SVC_DEVICE_INFO        ((uint16_t)0x180A)

// ---------------------------------------------------------------------------
// Gears. A gear is a virtual gear ratio, applied by telling the trainer the
// wheel is bigger or smaller than it is: the sim physics turns flywheel speed
// into road speed through the wheel size, so at the same cadence a bigger
// virtual wheel is a faster virtual road - exactly what a bigger gear does.
// The app's grade goes through untouched.
// ---------------------------------------------------------------------------
// The chain stays here. Every virtual ratio is relative to it.
static constexpr float kRealChainring = 34.0f;
static constexpr float kRealCog       = 17.0f;

// The 32-gear mode has no cassette to copy, so it spreads its own range in
// equal ratio steps - (6.0 - 0.5) / 31, about 0.177 a shift - and wider than
// a real 2x at both ends.
static constexpr float kSmoothMinRatio = 0.5f;
static constexpr float kSmoothMaxRatio = 6.0f;

// A single-crossover walk through a Shimano Ultegra 50/34 chainring set and
// 11-34 12-speed cassette, 34/34 to 50/11 (the rings/cogs Ui::nearestCombo()
// rounds a ratio to for display): every small-ring cog ascending, then jump
// the front once and take every big-ring cog the small ring hasn't already
// covered - the exact transition point (34/14 -> 50/19) E-TUBE's own
// synchronized-shift table uses. Merge-sorting all 24 raw combinations by ratio instead makes the
// front zig-zag repeatedly through the rings' overlap; real synchro-shift
// firmware (and this) crosses over exactly once per direction, at the cost
// of never using the middle combinations on the skipped side (50/21-50/27,
// 34/11-34/13). Used verbatim as gear 1..16 in the low-count mode.
static constexpr float kRealGearRatios[16] = {
    34.0f / 34.0f, 34.0f / 30.0f, 34.0f / 27.0f, 34.0f / 24.0f, 34.0f / 21.0f,
    34.0f / 19.0f, 34.0f / 17.0f, 34.0f / 15.0f, 34.0f / 14.0f, 50.0f / 19.0f,
    50.0f / 17.0f, 50.0f / 15.0f, 50.0f / 14.0f, 50.0f / 13.0f, 50.0f / 12.0f,
    50.0f / 11.0f,
};

// Two supported gear counts, switched from the ride screen: the real cassette
// above, or kSmoothMinRatio..kSmoothMaxRatio spread over 32 gears.
static constexpr int   kGearCountLow = 16;
static constexpr int   kGearCountHigh = 32;
static constexpr int   kDefaultGearCount = kGearCountLow;
static_assert(sizeof(kRealGearRatios) / sizeof(kRealGearRatios[0]) == kGearCountLow,
              "kRealGearRatios must have exactly kGearCountLow entries");
static constexpr float kGradeClampLow    = -10.0f;
static constexpr float kGradeClampHigh   = 20.0f;

// Until changed on the profile screen.
static constexpr float kDefaultRiderKg = 89.0f;
static constexpr float kDefaultBikeKg  = 8.0f;
static constexpr float kDefaultRiderCm = 181.0f;
static constexpr float kDefaultFtpW = 220.0f;

// Fallbacks until the app sends its own simulation parameters.
static constexpr float kDefaultCrr = 0.00415f;
static constexpr float kDefaultCw  = 0.51f;

// D-Fly channels, 1-based exactly as numbered in E-TUBE.
static constexpr int kDefaultUpChannel   = 2;
static constexpr int kDefaultDownChannel = 1;
