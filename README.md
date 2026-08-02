# LD2402

Arduino / PlatformIO driver for the **Hi-Link HLK-LD2402**, a 24GHz mmWave
radar that detects moving, micro-moving (breathing-level stillness) and
static human presence, with distance and per-gate signal strength.

Full control, not just an on/off pin: presence, distance, all 32 energy
gates, max-distance and per-gate threshold configuration, auto-calibration,
auto-gain, and saving settings to the sensor's own flash — everything the
vendor's PC tool and the third-party Home Assistant/ESPHome components
expose, as a plain library you can drop into any sketch.

Self-contained: the only dependency is Arduino's `Stream` interface, so it
works on hardware UART, a second hardware UART (ESP32's `Serial1`/`Serial2`),
or `SoftwareSerial`.

## Contents

- [Read this first](#read-this-first) — the five things that catch everyone
- [Why not just use the IO pin?](#why-not-just-use-the-io-pin)
- [Hardware](#hardware) · [Install](#install) · [Quick start](#quick-start)
- [Two ways to feed it data](#two-ways-to-feed-it-data)
- **API reference**
  - [Live readings](#live-readings-from-whichever-mode-the-sensor-is-streaming)
  - [Moving vs still: the sensor does not tell you](#moving-vs-still-the-sensor-does-not-tell-you)
  - [Report mode](#report-mode)
  - [Settings that persist](#settings-that-need-to-persist--one-call-each)
  - [Raw configuration](#raw-configuration-all-blocking--wrap-a-batch-in-enableconfigendconfig-yourself)
  - [Keeping your sketch alive: `onIdle()`](#keeping-the-rest-of-your-sketch-alive-onidle)
- [What the "gates" are](#what-the-gates-are)
- **Protocol notes**
  - [Behaviour that will surprise you](#behaviour-that-will-surprise-you)
  - [Wire format](#wire-format)
- [Related](#related) · [License](#license)

## Read this first

Five things about this module that aren't in the manual, cost real debugging
time to find, and change how you write your code. Each links to the detail.

| # | The trap | What to do |
|---|---|---|
| 1 | **The sensor never tells you moving vs still.** The status byte is a presence bit plus an unrelated flag — reading it as an enum gives you three separate bugs at once. | Call `cacheThresholds()` once after `begin()`; the library derives it. [Detail](#moving-vs-still-the-sensor-does-not-tell-you) |
| 2 | **Leaving config mode does not save anything.** Values set without an explicit save read back fine, then vanish on the next power cycle. | Use the `setAndSave*` calls. [Detail](#behaviour-that-will-surprise-you) |
| 3 | **Gate 0 is dead code** — the module never evaluates it, so nothing closer than 0.7 m is detectable and no setting changes that. | Design around it. [Detail](#behaviour-that-will-surprise-you) |
| 4 | **Deep-stillness detection only works between 2.8 m and 4.9 m.** Hard-wired to four gates. Outside that band you get motion only. | Place the sensor accordingly. [Detail](#behaviour-that-will-surprise-you) |
| 5 | **Output mode is not persistent.** After the module loses power it is back to text mode, and engineering data stops. | Re-send `setEngineeringMode(true)` when the module reappears. [Detail](#behaviour-that-will-surprise-you) |

Also worth knowing before you wire anything: **give the sensor its own UART**
— sharing one with a debug console breaks it in both directions, and the
reasons are [not obvious](#two-ways-to-feed-it-data).

## Why not just use the IO pin?

The sensor also has a plain presence-out pin (HIGH/LOW). If that's all you
need, wire that pin to any GPIO and skip this library entirely. This library
is for when you want the *rest* of what the sensor can do — distance,
signal-strength-by-gate, remote calibration — which only exists over UART.

## Hardware

| Pin (J2) | Function |
|---|---|
| V | Power, **3.0–3.6V typical (check your board — see below)** |
| IO | Presence out, HIGH/LOW (unused by this library — wire T/R instead) |
| G | Ground |
| T | UART TX (sensor → your board's RX) |
| R | UART RX (your board's TX → sensor) |

- **Fixed baud rate: 115200, 8N1.** Not configurable on the sensor side.
- **Give it a supply that can source ~50mA average, more on peaks.** A weak
  regulator (e.g. some USB-serial adapters' onboard 3.3V rail) can make an
  otherwise-fine module look dead.
- The datasheet lists an optional 4.5–5.5V input via an add-on LDO, but notes
  it as *"default not posted"* — i.e. not fitted at the factory. Assume
  3.3V-only unless you've confirmed your specific board has that LDO.

## Install

### Arduino IDE

1. Download this repo as a ZIP: green **Code** button on the
   [GitHub page](https://github.com/g0urav2410/LD2402) → **Download ZIP**.
   (Don't unzip it — the IDE wants the .zip file itself.)
2. In the Arduino IDE: **Sketch → Include Library → Add .ZIP Library…**,
   pick the file you just downloaded.
3. That's it — the library (and its examples) now show up like any other
   installed library.
4. Try it: **File → Examples → LD2402 → BasicPresence**, wire the sensor
   per the table above (T→your board's RX, R→your board's TX, plus power
   and ground), pick your board under **Tools → Board**, and upload.
5. Open **Tools → Serial Monitor** (115200 baud) to see presence/distance
   printed live.

If you'd rather install it once for every sketch instead of per-project:
clone or download this repo into your Arduino **libraries** folder directly
(*File → Preferences* shows the "Sketchbook location" — the library goes in
a `libraries` subfolder there), then restart the IDE.

### PlatformIO

Add to `platformio.ini`:
```ini
lib_deps = https://github.com/g0urav2410/LD2402.git
```

## Quick start

```cpp
#include <LD2402.h>

LD2402 radar;

void setup() {
    Serial.begin(115200);
    radar.begin(Serial);
}

void loop() {
    radar.loop();
    if (radar.presence()) {
        Serial.println(radar.distanceCm());
    }
}
```

See `examples/BasicPresence` for the full version, `examples/FullControl` for
engineering mode + calibration + all 32 energy gates, and
`examples/NonBlockingUI` for keeping a display or animation running during the
library's slow configuration calls.

## Two ways to feed it data

- **`radar.loop()`** — the sensor has the UART to itself. Call this every
  `loop()` iteration and read the live getters below. Simplest option, and
  strongly recommended (see the warning).
- **`radar.feedByte(uint8_t)`** — feed the parser one byte at a time yourself.
  `radar.midFrame()` reports whether a binary engineering frame is in
  progress. Only reach for this if you have a specific reason to touch each
  byte first.

> **⚠️ Give the sensor its own UART. Don't share it with a log/debug console.**
> Two hard-won lessons from real use, both worth avoiding:
> 1. **Your debug *output* on the shared TX goes into the sensor's RX** and
>    interferes with it accepting commands. This is why the ESPHome component
>    tells you to set `baud_rate: 0` (disable UART logging). Keep TX quiet.
> 2. **The sensor's binary energy bytes will masquerade as console *input*.**
>    An engineering frame's body is arbitrary bytes — feed those to a
>    command-interpreting console and a stray `0x78`, `'x'`, or a digit gets
>    executed as a command. If you must share the wire, route by frame
>    structure and treat every non-frame byte as noise to discard — never as a
>    command. Simplest is to not share it at all.

## API reference

### Live readings (from whichever mode the sensor is streaming)

| Call | Returns |
|---|---|
| `read()` | `Reading{presence, moving, still, connected, distanceCm}` — everything below, in one call |
| `presence()` | `bool` — someone detected |
| `isMoving()` / `isStill()` | `bool` — **derived here, not reported by the sensor.** See below |
| `distanceCm()` | `uint16_t` — distance to the target |
| `haveEnergyGates()` | `bool` — true once an engineering frame has arrived |
| `triggerEnergyDb(gate)` / `motionlessEnergyDb(gate)` | `float` dB, gate 0–15, near → far |
| `connected()` | `bool` — data received in the last 2s |
| `lastUpdateMs()` | `unsigned long` — `millis()` of the last reading |
| `cacheThresholds()` | reads all 32 thresholds so `isMoving()` can classify — call once after `begin()` |
| `haveThresholdCache()` | `bool` — whether that has happened |

#### Moving vs still: the sensor does not tell you

This is the single most important thing to understand about this module, and
it is not in the manual.

The engineering frame's first byte looks like a status enum. It isn't. It is:

```
state = presence;                    // 0 or 1
if (<reporting sub-mode>) state += 0x10;
```

So the only values that ever appear are **`0x00`, `0x01`, `0x10`, `0x11`** —
a presence bit, plus a flag about the module's own reporting mode that has
nothing to do with the room. Read it as "1 = moving, 2 = still" and you get
three separate bugs: `isStill()` can never be true (2 never arrives), a
moving target reads as neither whenever the sub-mode flag is set, and `0x10`
— which means **nobody is present** — is counted as presence because it is
non-zero.

And the distinction cannot be recovered from that byte, because inside the
module the motion and micro-motion chains are merged with a plain `OR` into
one presence bit before the frame is built.

So this library derives it instead, from the per-gate energies against the
per-gate thresholds — which is what the frame carries two energy arrays for:

```cpp
radar.cacheThresholds();     // once, after begin()
...
radar.isMoving();            // any gate 1-15 over its trigger threshold
radar.isStill();             // present, but not moving
```

Without `cacheThresholds()`, `isMoving()` falls back to reporting any
presence as movement — the safer of the two wrong answers, and no worse than
what you had before.

Gate 0 is excluded deliberately: the module never evaluates it (see
[Protocol notes](#protocol-notes)), so including it would let near-field
clutter set a flag no configuration change could clear.

### Report mode

| Call | Effect |
|---|---|
| `setEngineeringMode(bool on)` | **Preferred** — one call, manages its own config session. `false` = plain "OFF"/"distance : NN" text (factory default). `true` = binary frames with distance + all 32 energy gates. |
| `setOutputMode(bool engineering)` | Raw version — you wrap `enableConfig()`/`endConfig()` yourself. Only reach for this inside a batch of other config calls. |

### Settings that need to persist — one call each

These set the value **and** commit it to the sensor's own flash, each managing
its own config session. This is what you want almost all the time — a value
set without saving reverts the moment the sensor loses power.

Each of these sends the explicit save command (`0x00FD`) before leaving
config mode. Exiting config mode alone does **not** persist anything; see
[Protocol notes](#protocol-notes).

| Call | Sets |
|---|---|
| `setAndSaveMaxDistanceMeters(float)` | Max detection range, 0.7–10.0m |
| `setAndSaveDisappearDelaySec(uint16_t)` | How long presence is held after the target leaves |
| `setAndSaveTriggerThresholdDb(gate, db)` | One gate's trigger threshold, gate 0–15 |
| `setAndSaveMotionlessThresholdDb(gate, db)` | One gate's motionless/still threshold, gate 0–15 — automatically routes gate 15 through its known flash-persistence quirk, no special handling needed from you |
| `saveAllThresholds(motionDb[16], microDb[16])` | All 32 thresholds at once, in one efficient session (pass `nullptr` for either array to skip it) — use this instead of 16 individual calls when applying a full set |

### Raw configuration (all blocking — wrap a batch in `enableConfig()`/`endConfig()` yourself)

Reach for these only when you're batching several changes in one session for
efficiency (like `saveAllThresholds()` does internally), or reading a current
value. For a single change, the `setAndSave*` calls above are simpler.

| Call | Notes |
|---|---|
| `enableConfig()` / `endConfig()` | Required bracket around every call below. Retries internally on entry. |
| `readFirmwareVersion(String&)` / `readSerialNumber(String&)` | |
| `setMaxDistanceMeters(float)` / `readMaxDistanceMeters(float&)` | 0.7–10.0m, live only |
| `setDisappearDelaySec(uint16_t)` / `readDisappearDelaySec(uint16_t&)` | Live only |
| `setTriggerThresholdDb(gate, db)` / `readTriggerThresholdDb(gate, db&)` | gate 0–15, live only |
| `setMotionlessThresholdDb(gate, db)` / `readMotionlessThresholdDb(gate, db&)` | gate 0–15, live only |
| `readPowerInterference(uint8_t&)` | 0 not run, 1 clear, 2 interference detected |
| `startCalibration(trigger, hold, micro)` | Auto-generates thresholds for the room. Factors default 3. |
| `calibrationProgress(uint8_t&)` | 0–100, poll until 100 |
| `startAutoGain()` / `autoGainDone(timeoutMs)` | Corrects a saturated front-end. `autoGainDone` waits for the sensor's own completion push — it isn't a normal ACK. |
| `saveParameters()` | Commits whatever's currently set to the sensor's own flash. Prefer `setAndSave*` above for a single value. |
| `readParameterRaw(id, value&)` / `setParameterRaw(id, value)` | Escape hatch for any parameter ID not wrapped above |
| `onIdle(fn)` | Runs `fn` while any of the above is waiting on the module, so your display/UI doesn't freeze — see [below](#keeping-the-rest-of-your-sketch-alive-onidle) |

Every blocking call takes an optional `timeoutMs` (default 1000ms, longer for
`autoGainDone`).

### Keeping the rest of your sketch alive: `onIdle()`

Every configuration call above blocks: it sends a command and waits for the
module to reply. On a single-threaded board — any AVR, an ESP8266, a
single-task ESP32 sketch — **nothing else in your sketch runs during that
wait**. A display freezes, an animation stops, a button goes unread.

`onIdle()` hands that time back. Give it a function, and the driver calls it
repeatedly wherever it would otherwise just be waiting.

```cpp
void keepUiAlive() {
    static unsigned long last = 0;
    if (millis() - last < 1000) return;   // <- the important line
    last = millis();
    redrawDisplay();
}

void setup() {
    radar.begin(Serial);
    radar.onIdle(keepUiAlive);            // that's the whole integration
}
```

That's it. One call, once, and every slow operation — reading settings, saving
thresholds, calibration, auto-gain — stops freezing your sketch.

Run **`examples/NonBlockingUI`** to see it: it blinks the built-in LED while
reading all 32 thresholds, and `n` toggles the hook on and off so you can watch
the LED freeze and unfreeze.

#### Writing the callback

**It is called about 8,000 times a second.** That is not an exaggeration —
while waiting for a reply the driver spins in a tight loop, and your function
runs on every pass.

So the first line must be a cheap test that usually returns immediately. The
`if (millis() - last < interval) return;` shape above is the standard way; in a
measured run of a bulk save the callback was entered **50,012 times and did real
work 12 times**. Everything else was that early return.

Get this wrong and you make things worse: a callback costing 1ms, called 8,000
times, adds 8 seconds to a 6-second operation.

Two rules:

- **Keep it cheap.** Early-out first, work second.
- **Never call `radar.*` from inside it.** A command/response exchange is in
  progress the entire time; re-entering the driver will corrupt it.

Safe things to do: refresh a display, toggle a pin, feed a hardware watchdog,
step an animation. Unsafe: anything that talks to this sensor, anything that
blocks, anything doing heavy I/O.

#### Why the hook is where it is

Worth knowing if you're wondering whether it covers the case you care about.
Measured on a real HLK-LD2402, a `saveAllThresholds()` totalling **6.2s**:

| Phase | Time |
|---|---|
| The 31 threshold writes | ~0.6s (19ms each) |
| Flash commit, config mode opening and closing | **~5.6s** |

The writes are under 10% of it. So the hook isn't called "between writes" — it
sits at the driver's actual wait points, including the fixed settle delays,
which is why a single `onIdle()` covers every blocking call in the library
rather than one of them.

#### Compatibility

`onIdle()` is entirely optional. Without it, the driver behaves exactly as it
always has — the internal wait is a plain `yield()`, unchanged. Nothing about
the protocol, the timing, or any other call is affected by adding or omitting
it, so upgrading is safe.

### Configuring while engineering data is streaming

You *can* enter config mode (to read/change settings, calibrate) while the
sensor is mid engineering-stream — but the enable command's ACK is easily lost
in that ~775 byte/s flood. `enableConfig()` is therefore **deadline-based**:
give it a generous timeout (e.g. `enableConfig(2500)`) and it re-sends the
enable request until it breaks in — the moment the sensor accepts it, the
stream stops and the ACK comes back cleanly. `endConfig()` likewise retries,
so a missed exit can't strand the sensor silent in config mode. After you exit,
streaming resumes on its own. Verified reading/writing settings and running
calibration live, without dropping engineering data for more than the config
window.

## What the "gates" are

The sensor divides range into fixed **~0.7 m distance slices called gates**,
gate 0 nearest. An engineering frame reports, per gate, the reflected signal
strength — and each gate has two independent detection thresholds:

- **motion** gate/threshold — *moving* targets at that distance (walking)
- **micro-motion** gate/threshold — *still* targets (sitting, breathing)

Detection at a gate fires when its energy crosses its threshold. Tuning
per-gate thresholds lets you, e.g., desensitise far gates so movement *behind*
a wall stops triggering, or raise micro sensitivity at the one distance where
someone actually sits.

## Protocol notes

Things the manual doesn't say outright. Reverse-assembled from the official
HLK-LD2402 user manual (v1.08) — Hi-Link publishes no separate protocol PDF
— found by testing, and most of it since confirmed against a full
disassembly of firmware v3.3.5.

Split three ways on purpose: **behaviour** changes how you write your code
and is worth reading start to finish; **wire format** is lookup material for
when a packet capture doesn't match expectations; and there is exactly one
outright **firmware bug** that needs a workaround.

### Behaviour that will surprise you

- **Exiting config mode does NOT save anything.** Every persisting call in
  this library sends `0x00FD` (`saveParameters()`) before `endConfig()`. An
  earlier version relied on `endConfig()` committing by itself; values set
  that way read back correctly and then reverted after a power cycle.
  Confirmed in the disassembly: end-config calls two re-apply routines and
  touches neither flash nor the deferred-save flag, and `0x00FD` only
  *requests* the save — the module's main loop performs the erase and write
  afterwards, so a delay before leaving config mode is required.
- **Gate 0 is dead code.** Both detection loops start at gate 1.
  `TriggerGate0Threshold` and `MotionlessHoldGate0Threshold` have no effect
  whatsoever. If detection closer than 0.7m is failing, this is why, and no
  configuration changes it.
- **`HoldGateNThreshold` (IDs `0x0020`–`0x002F`) is not implemented.** Those
  IDs hit a stub returning constant 0 — no storage, no reader. This is why
  the vendor tool shows them all as `0.00`. Writing them is discarded
  silently. Only Trigger and Motionless-hold are real.
- **Deep-stillness detection only covers gates 4–7 (2.8–4.9m).** The
  long-window tracker that keeps a motionless person detected is hard-wired
  to those four gates. Outside that band you get motion detection only.
  Its threshold is also adaptive and not configurable, tracking a slow
  average of motion energy in the same gate — so persistent motion-band
  noise there (a fan, a curtain) desensitises still detection.
- **Calibration progress reads 0% for the first ~14%** of the run — the
  countdown starts at 31,360 but the percentage is computed against 26,880.
  Don't treat "still 0%" as stalled.
- **Moving and still are not mutually exclusive** in the module's own logic.
  They are one signal under two different filter time constants, so a person
  who has just stopped walking registers in both.
- **Engineering data only streams after `endConfig()`** — enabling it while
  still in config mode has no visible effect.
- **Output mode is not persistent.** It lives in the module's RAM, unlike
  thresholds and distances which are in its flash. After a module power
  cycle it returns to text mode and `setEngineeringMode(true)` must be sent
  again.
### Wire format

Reference material — the library already handles all of it. Useful when
comparing against a packet capture or writing your own parser.

- **Baud rate 115200, 8N1, fixed.** USART2 on the module's own MCU.
- **Energy gates:** the 128-byte body in an engineering frame is 32×4-byte
  little-endian values — 16 motion gates followed by 16 micro-motion gates,
  matching the 16 threshold IDs of each (`0x0010`–`0x001F` and
  `0x0030`–`0x003F`).
- **Thresholds on the wire are linear power, not dB.** `raw = 10^(dB/10)`,
  and back with `dB = 10·log10(raw)`. Writing the dB number directly gives
  you a threshold around 15 dB — wildly over-sensitive, and an easy way to
  end up with a sensor that triggers on everything.
- **ACK frames echo the command word with `+0x0100` set.** Auto-gain's
  completion report is the one exception — it arrives unprompted, carrying
  the bare word `0x00F0`, not an ACK to anything you sent.
- **Frame delimiters:** commands `FD FC FB FA … 04 03 02 01`; engineering
  frames `F4 F3 F2 F1 … F8 F7 F6 F5`.

### One outright firmware bug

- **Gate 15's micro threshold (`0x003F`) doesn't persist via the normal
  set-then-`saveParameters()` sequence** — confirmed on firmware v3.3.5,
  deterministic, 100% reproducible. The write itself succeeds and applies
  live; it's specifically the flash-commit step that silently fails whenever
  `0x003F` was touched, while every other parameter (including gate 15's own
  motion threshold, `0x001F`) saves and persists normally. Workaround that's
  been verified across a real power cycle: `enableConfig()` → set the value
  → **wait ~200ms** → read it back → set it again → `endConfig()`, **without**
  calling `saveParameters()` at all. See
  `LD2402::persistGate15MotionlessThresholdDb()` for the implementation. Hi-Link
  support confirmed this workaround but couldn't explain *why* it's needed —
  their explanation didn't match the documented protocol, so treat this as
  empirically verified rather than fully understood.

## Related

- [Presently](https://github.com/g0urav2410/Presently) — a standalone
  presence-sensor product built on this driver: ESP32-C3 firmware with an
  HTTP API, MQTT/Home Assistant discovery, and a Flutter app with live
  per-gate tuning. The current reference integration.
- [Clockwise](https://github.com/g0urav2410/Clockwise) — the smart-clock
  project this library was originally built for. The sensor has since been
  split out of it into Presently, so the integration there is historical.
- **Firmware reverse-engineering writeup** — a full disassembly of
  LD2402 firmware v3.3.5 (signal chain, all 24 commands, the parameter map,
  calibration and persistence). Most of the protocol notes above trace back
  to it. Not published; kept alongside this repo.

## License

MIT — see [LICENSE](LICENSE).
