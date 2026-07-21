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

**PlatformIO** — add to `platformio.ini`:
```ini
lib_deps = https://github.com/g0urav2410/LD2402.git
```

**Arduino IDE** — download this repo as a ZIP, then *Sketch → Include
Library → Add .ZIP Library…*.

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

See `examples/BasicPresence` for the full version, and
`examples/FullControl` for engineering mode + calibration + all 32 energy
gates.

## Two ways to feed it data

- **`radar.loop()`** — the sensor has the UART to itself. Call this every
  `loop()` iteration and read the live getters below. Simplest option.
- **`radar.feedByte(uint8_t)`** — something else shares the same UART (a
  debug console, another device). You read the incoming bytes yourself and
  decide which ones are the radar's; feed those to `feedByte()` one at a
  time. `radar.midFrame()` tells you whether a binary engineering frame is
  currently in progress, so a byte-router can keep sending bytes here
  regardless of their value until the frame completes. This is the pattern
  used in the [Clockwise](https://github.com/g0urav2410/AjantaClock) project,
  which shares one UART between this sensor and a USB debug console — see
  its `main.cpp` for a worked example of the routing logic.

## API reference

### Live readings (from whichever mode the sensor is streaming)

| Call | Returns |
|---|---|
| `presence()` | `bool` — someone detected (moving or still) |
| `isMoving()` / `isStill()` | `bool` — which kind of presence |
| `distanceCm()` | `uint16_t` — distance to the target |
| `haveEnergyGates()` | `bool` — true once an engineering frame has arrived |
| `motionEnergyDb(gate)` / `microEnergyDb(gate)` | `float` dB, gate 0–15, near → far |
| `connected()` | `bool` — data received in the last 2s |
| `lastUpdateMs()` | `unsigned long` — `millis()` of the last reading |

### Report mode

| Call | Effect |
|---|---|
| `setOutputMode(bool engineering)` | `false` = plain "OFF"/"distance : NN" text (factory default). `true` = binary frames with distance + all 32 energy gates. **Takes effect once `endConfig()` is called** — the sensor doesn't stream while still in config mode. |

### Configuration (all blocking — wrap a batch in `enableConfig()`/`endConfig()`)

| Call | Notes |
|---|---|
| `enableConfig()` / `endConfig()` | Required bracket around every call below. Retries internally on entry. |
| `readFirmwareVersion(String&)` / `readSerialNumber(String&)` | |
| `setMaxDistanceMeters(float)` / `readMaxDistanceMeters(float&)` | 0.7–10.0m |
| `setDisappearDelaySec(uint16_t)` / `readDisappearDelaySec(uint16_t&)` | How long presence is held after the target leaves, 0–65535s |
| `setMotionThresholdDb(gate, db)` / `readMotionThresholdDb(gate, db&)` | gate 0–15 |
| `setMicroThresholdDb(gate, db)` / `readMicroThresholdDb(gate, db&)` | gate 0–15 |
| `readPowerInterference(uint8_t&)` | 0 not run, 1 clear, 2 interference detected |
| `startCalibration(trigger, hold, micro)` | Auto-generates thresholds for the room. Factors default 3. |
| `calibrationProgress(uint8_t&)` | 0–100, poll until 100 |
| `startAutoGain()` / `autoGainDone(timeoutMs)` | Corrects a saturated front-end. `autoGainDone` waits for the sensor's own completion push — it isn't a normal ACK. |
| `saveParameters()` | Commits to the sensor's own flash. Everything above is otherwise live-only and reverts on power loss. |
| `readParameterRaw(id, value&)` / `setParameterRaw(id, value)` | Escape hatch for any parameter ID not wrapped above |

Every blocking call takes an optional `timeoutMs` (default 1000ms, longer for
`autoGainDone`).

## Protocol notes

Reverse-assembled from the official HLK-LD2402 user manual (v1.08); Hi-Link
does not publish a separate protocol PDF. A few things the manual doesn't
say outright, found by testing:

- **Engineering data only streams after `endConfig()`** — enabling it while
  still in config mode has no visible effect.
- Energy gates: the 128-byte body in an engineering frame is 32×4-byte
  values — 16 motion gates followed by 16 micro-motion gates, matching the
  16 threshold IDs of each (`0x0010`–`0x001F` and `0x0030`–`0x003F`).
- dB conversion both ways: `dB = 10·log10(raw)`, `raw = 10^(dB/10)`.
- ACK frames echo the command word with `+0x0100` set. Auto-gain's
  completion report is the one exception — it arrives unprompted, carrying
  the bare word `0x00F0`, not an ACK to something you sent.

## Related

- [Clockwise](https://github.com/g0urav2410/AjantaClock) — the smart-clock
  project this library was built for; see its firmware for a real
  shared-UART integration, plus an HTTP API and Flutter app built on top of
  this driver's data.

## License

MIT — see [LICENSE](LICENSE).
