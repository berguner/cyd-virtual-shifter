# CYD Virtual Shifter

Virtual shifting for a **Wahoo KICKR v5 (2020)**, driven by **Shimano 12-speed
Di2 hood buttons**, running on a **Cheap Yellow Display** (ESP32-2432S028R,
two-USB-port / ST7789 variant).

```
  KICKR v5  <--BLE--  CYD  --BLE-->  Zwift / MyWhoosh
   (Wahoo proprietary) |  (FTMS + Cycling Power)
                       |
          +------------+------------+
          |                         |
  Di2 R8150 derailleur      cadence sensor
    (D-Fly channels)        (speed + cadence)
```

## Why a bridge and not a plugin

The KICKR v5 is a dead end for native virtual shifting:

* Wahoo ruled out Zwift Virtual Shifting for the v4/v5 — the hardware can't run
  the protocol, and there will be no firmware update.
* The v5 doesn't implement FTMS either. It speaks the standard Cycling Power
  Service for telemetry plus Wahoo's own control point for resistance.

So nothing in Zwift or MyWhoosh can shift gears on it. The only way is to sit
between the trainer and the app — which is exactly what QZ and SHIFTR do. This
firmware does the same thing on a £12 display board.

## How a gear works here

A gear is a **virtual gear ratio**, applied through the trainer's wheel size.
The KICKR's sim mode turns flywheel speed into road speed with its configured
wheel circumference, then resists as that speed, grade, weight, rolling and air
resistance demand. Tell it the wheel is bigger and, at the same cadence, it
thinks you are going faster — which is exactly what a bigger gear does:

```
wheel size = 2146 mm × virtual ratio / real ratio (34/17)
```

The chain stays on **34/17**. In 16-gear mode virtual ratios span a Shimano
Ultegra 50/34 × 11-34 12-speed, **34/34 (1.00) to 50/11 (4.55)**; in 32-gear
mode **0.50 to 6.00** — see [Screen](#screen) below for how the steps in
between are chosen. The app's grade goes to the trainer untouched; the trainer
does all the physics.

Rough feel on the flat at 77 rpm, 97 kg: 34/34 ≈ 10 km/h, 20 W; 34/17 ≈
20 km/h, 65 W; 50/13 ≈ 38 km/h, 340 W.

The wheel size persists on the KICKR between sessions, and a wrong one left by
another app silently scales every watt (this is what made the trainer feel
three times too easy, even paired directly). It is set on every connect.

Reported power and cadence are the trainer's own numbers, relayed untouched.
Nothing about your ride data is faked.

Speed is reported but nothing reads it: Zwift and MyWhoosh model speed from
power, rider weight and gradient. It is carried because the Indoor Bike Data
payload is *positional* — omitting the field via the More Data flag is legal
FTMS, but MyWhoosh then read cadence as speed and power as cadence, and showed
zero watts. The value comes from the trainer's own wheel-revolution counters.

## Cadence source

The KICKR v5 infers cadence from flywheel speed, which lags and drifts. If a
BLE speed/cadence sensor is in range it is picked up automatically, and the
**CAD:** button on the right of the screen switches between the two.

The choice follows through to the app, not just the display: when the sensor is
selected, its crank counters are spliced into the Cycling Power packet on the
way past. Both services count crank events in 1/1024 s, so the numbers carry
over untouched and Zwift derives cadence exactly as it would from the sensor
paired directly.

Cadence counts as stopped once no revolution has arrived for two revolutions'
worth of time at the last cadence (1.3 s minimum, 2 s maximum — about 1.4 s at
85 rpm). The display, the FTMS cadence and the Cycling Power counters all use
that one rule. For the counters, trainer and sensor both keep repeating their
last values when you stop, which apps read as "no news" and hold the old
cadence for several seconds; so the event time sent to the app is moved on
without a new revolution, which reads as zero straight away.

If the sensor is selected but goes quiet for 4 s while the trainer still sees
pedalling, cadence falls back to the trainer rather than dropping out
mid-ride, and the button turns amber (`CAD: NO SENSOR`) so the reason is
visible. A sensor going quiet because the cranks stopped is not a dropout.

## ERG

In ERG mode the gears do nothing — the workout owns the resistance.

The trainer only goes into ERG once you are pedalling with the flywheel
above 8 km/h, and drops back to sim after 4 s without cadence. A KICKR v5 put
into ERG with its flywheel stopped went on reporting zero speed and power
every time it was in ERG afterwards, braking as if you had stalled, until it
was power-cycled. If ERG ever feels seized with power reading 0 while you
pedal, power-cycle the KICKR.

After re-establishing sim mode the trainer's acknowledgement is awaited and
the command resent if none comes — a write can go through without the
trainer taking it, and one that did was followed by exactly that stall.

MyWhoosh switches ERG off and back on with the same sim-parameters write, and
follows the switch-on with a target only if the target changed since it last
sent one — so switching ERG back on within a workout step used to do nothing.
A sim-parameters write that ends ERG therefore only pauses it; an identical
one at least 3 s later resumes it at the kept target. Zwift re-sends its
target every 20 s and streams grade with every change, so this never
triggers there.

## One-time setup

### 1. E-TUBE

In **E-TUBE PROJECT Cycling**, assign the two hood-top buttons to **D-Fly
channels**. Nothing is broadcast over BLE otherwise.

Default mapping in `include/Config.h`:

| D-Fly channel | Action |
|---|---|
| 1 | shift down (easier) |
| 2 | shift up (harder) |

A double-press shifts two gears. Long press is ignored.

### 2. Build and flash

```sh
~/.platformio/penv/bin/pio run -e cyd2usb
~/.platformio/penv/bin/pio run -e cyd2usb -t upload --upload-port /dev/cu.usbserial-XXXX
```

Flash over the **micro-USB** port — the USB-C port has no CC resistors.
`upload_speed` is pinned to 115200 on purpose; faster rates corrupt the stub.

### 3. Pick your devices

First boot lands on the device picker. It scans continuously and lists whatever
it finds, tagged with a guess at the role:

```
TAP A DEVICE TO CONNECT                    7 found
[TRAIN] KICKR 1E2F                             -54
[SHIFT] RDR8150-4A21                           -61
[ CAD ] COOSPO BK467                           -68
[  HR ] TICKR 9C21                             -63
[  ?  ] 4c:1d:96:aa:bb:cc                      -88
[ UP ] [ DOWN ] [      RIDE >      ]
```

Tap a row to connect it. The role tag is only a guess from the advertisement —
what actually decides is the service the device exposes once connected, so a
row tagged `?` is still worth tapping. That is the fallback if your sensor
advertises nothing recognisable.

Heart rate straps are tagged `HR` and deliberately not connectable: probing one
would open a real link and take its single connection slot away from Zwift for
no benefit. Pair the strap with the training app directly.

Picked devices turn green and are remembered, so later boots go straight to the
ride screen and reconnect on their own. **Tap a remembered row again to forget
it.** The picker is always one tap on the ride screen's header away.

### 4. Pair

In Zwift or MyWhoosh pair **`CYDShift`**, not the KICKR.

The Di2 derailleur only advertises when awake, so press a shift button if it
does not show up in the picker.

Pair it as both *Power Source* and *Controllable*. Ignore the real KICKR in the
pairing list — if you pair that directly, you get no shifting.

## Screen

```
[KICKR] [ DI2] [ CAD] [ APP] [(o)]  <- tap to reopen the picker | profile
[  16 GEARS  ]       POWER [DEBUG]  <- switch 16 / 32 | open the debug screen
  9                  243
                     [ CAD: SENSOR ]  <- tap to switch source
 34x14 2.43          88
                     GRADE
[ - EASIER ] [ HARDER + ]   3.0%
```

Pair your heart rate strap with Zwift directly; it is not relayed here.

`34x14 2.43` is the virtual ratio and the nearest real Ultegra combination;
in 32-gear mode only the ratio is shown.

**16 or 32 gears.** 16 gears walk a real Ultegra 50/34 x 11-34 12-speed
cassette: every small-ring cog ascending, then the front crosses over once
and every remaining big-ring cog ascending - the same single transition point
(34/14 -> 50/19) E-TUBE's own synchronized-shift table uses. Sorting all 24
raw combinations by ratio instead (what this used to do) makes the front
zig-zag repeatedly through the rings' overlap, which feels wrong even though
each ratio is real. 32 has no real cassette to copy, so it spreads 0.50 to
6.00 uniformly instead: every shift adds 0.177 to the ratio, and the range is
wider than any real 2x at both ends. Switching lands on the gear nearest the ratio you are in, so the
resistance does not jump under you mid-ride.

The top of the 32-gear range needs a trainer wheel size of 6.4 m, close to
the 6553.5 mm the Wahoo command can carry; with a wheel circumference above
2184 mm on the profile screen, the top gears stop at that ceiling.

The two bottom buttons are a manual backup for the Di2 input; hold for
auto-repeat.

**Profile.** The head-and-shoulders button in the top-right corner opens rider
weight, bike weight, wheel circumference and rider height, each with `-`/`+`
(hold to repeat; after a couple of seconds a hold moves ten steps at a time).
Nothing changes until `< RIDE`: the values are then stored on the device and
sent to the trainer - at once in sim, or when ERG ends, since the sim-mode
write that carries the weight would end ERG. Height is stored but not used
yet: the apps send their own wind-resistance coefficient.

**Debug screen.** `DEBUG` shows both data streams live, refreshed four times a
second; `< RIDE` goes back, and the Di2 buttons keep shifting meanwhile.

| Section | Shows |
|---|---|
| KICKR > CYD | Each Cycling Power field as received — power, accumulated torque, wheel and crank counters with their event times — plus the flags, packet rate and age, and the speed and cadence derived from them. |
| APP > CYD | The latest FTMS control-point write in hex, the sim parameters (grade, crr, cw, wind), ERG state and target, and a count of each command the app has sent since it connected. |
| CYD > KICKR | Trainer mode, the last value sent with each Wahoo command (grade, wheel size, ERG watts, sim-mode kg/crr/cw), and the status the trainer answered each with — `01` accepted, red anything else, `--` no answer yet. |
| CYD > APP | Power, cadence and its source, and speed as sent in Indoor Bike Data. |

If ERG keeps dropping out, check the counts: an app that sends sim
parameters in between its target-power writes flips the trainer out of ERG
each time.

## Tuning

Everything lives at the bottom of `include/Config.h`:

| Setting | Default | Notes |
|---|---|---|
| `kGearCountLow` / `kGearCountHigh` | 16 / 32 | The two counts the on-screen button switches between. |
| `kRealChainring` / `kRealCog` | 34 / 17 | Where the chain actually sits. Change if you move it. |
| `kSmoothMinRatio` / `kSmoothMaxRatio` | 0.5 / 6.0 | Bottom and top of the 32-gear mode. |
| `kRealGearRatios` | 16 real 50/34 x 11-34 combos, single crossover | The 16-gear ladder, in order. Edit if your cassette or crossover point differs. |
| `kDefaultWheelMm` | 2146 | 700×30c. Editable on the profile screen. |
| `kDefaultRiderKg` / `kDefaultBikeKg` | 89 / 8 | Sent to the trainer's sim mode. Editable on the profile screen. |
| `kDefaultRiderCm` | 181 | Editable on the profile screen; not used yet. |
| `kDefaultUpChannel` / `kDefaultDownChannel` | 2 / 1 | Swap if the buttons feel backwards. |

Values are cached in NVS on first boot, so **after changing a default, erase
flash** (`pio run -t erase`) or the old value wins.

## Protocol notes

Reverse-engineering credit to the projects in the sibling directories.

**Wahoo control point** — `a026e005-0a7d-4ab3-97fa-f1500f9feb8b`, inside the
standard Cycling Power Service:

| Opcode | Command | Payload |
|---|---|---|
| `0x20` | unlock | `EE FC` — required before anything else works |
| `0x40` | resistance mode | `u16 LE`, `(1−fraction) × 16383` |
| `0x42` | ERG mode | `u16 LE` watts |
| `0x43` | sim mode | `u16 LE` kg×100, crr×1000, cw×1000 |
| `0x46` | sim grade | `u16 LE`, `(fraction+1) × 65535/2` |
| `0x48` | wheel circumference | `u16 LE` mm×10 — persists on the trainer |

Every command is answered on the same characteristic with
`01 <opcode> <status>`; status `01` is success. The v5 answers unlock with
`02` and works regardless.

**Shimano D-Fly over BLE** — service `000018ef-5348-494d-414e-4f5f424c4500`
(the suffix is ASCII `SHIMANO_BLE`), alternative `000018ff-…`, characteristic
`00002ac2-…` by indication. Payload is a header byte followed by one byte per
channel: `0x10` short press, `0x20` long, `0x40` double.

## Not yet verified on hardware

This compiles clean but has not been ridden. Expect to shake out:

* **Touch calibration.** Tapping a shift button prints `[touch] raw x,y` to
  serial; adjust `kTouchMinX`/`kTouchMaxX`/… in `src/Ui.cpp` if the buttons
  feel offset.
* **Di2 bonding.** If the derailleur refuses the connection, it may want a
  bonded pairing rather than the open connect used here.
* **Sensor discovery.** Role tags come from the advertisement, so a device that
  advertises nothing recognisable shows as `?`. Tapping it still works — the
  role is settled by probing the services. The serial log prints what each
  probe found.
