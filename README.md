# LD2402

An easy way to use the Hi-Link HLK-LD2402 presence sensor (a small radar
module that can tell if someone is in a room) from Arduino or PlatformIO.

It tells you:
- Is someone there? (`presence`)
- Are they moving, or sitting still? (`isMoving` / `isStill`)
- How far away are they? (`distanceCm`)

## Wiring

The module has 5 pins. You need 4 of them:

| Pin | Connect to |
|---|---|
| V | 3.3V power |
| G | Ground |
| T | Your board's RX pin |
| R | Your board's TX pin |
| IO | Not used — leave it disconnected |

Give it its own set of RX/TX pins. Don't share them with `Serial` /
`Serial.print()` debugging — the two will interfere with each other.

## Install

**Arduino IDE:** on the [GitHub page](https://github.com/g0urav2410/LD2402),
click the green **Code** button → **Download ZIP**. In the Arduino IDE go to
**Sketch → Include Library → Add .ZIP Library…** and pick the file you just
downloaded.

**PlatformIO:** add this line to `platformio.ini`:
```ini
lib_deps = https://github.com/g0urav2410/LD2402.git
```

## The simplest example

```cpp
#include <LD2402.h>

LD2402 radar;

void setup() {
    Serial1.begin(115200);   // the sensor's own serial port, not the USB one
    radar.begin(Serial1);
}

void loop() {
    radar.loop();
    if (radar.presence()) {
        Serial.print("Someone is ");
        Serial.print(radar.isStill() ? "sitting still, " : "moving, ");
        Serial.print(radar.distanceCm());
        Serial.println(" cm away");
    }
}
```

That's the whole thing you need for most projects: `begin()` once, `loop()`
every time round your `loop()`, then read whichever of the calls below you
need.

**One extra line makes `isMoving()`/`isStill()` accurate.** Without it they
still work, but fall back to a rougher guess. Add this once, right after
`begin()`:

```cpp
radar.cacheThresholds();
```

This reads the sensor's own sensitivity settings so the library can tell
moving and still apart properly. Call it again any time after you run
calibration (below), since that changes those settings.

## The main things you can ask it

| Call | What it gives you |
|---|---|
| `radar.presence()` | `true`/`false` — is anyone there |
| `radar.isMoving()` | `true`/`false` — are they moving |
| `radar.isStill()` | `true`/`false` — are they there but not moving |
| `radar.distanceCm()` | how far away, in centimetres |
| `radar.connected()` | `true` if the sensor has sent data in the last 2 seconds |
| `radar.read()` | all four of the above, in one go |

Using `read()` instead of calling each one separately:

```cpp
LD2402::Reading r = radar.read();
if (r.presence) {
    Serial.println(r.moving ? "moving" : "still");
    Serial.println(r.distanceCm);
}
```

## Making it more/less sensitive to distance

```cpp
radar.setAndSaveMaxDistanceMeters(6.0);      // ignore anything past 6m
radar.setAndSaveDisappearDelaySec(10);       // keep "presence" on for 10s
                                              // after someone leaves, instead
                                              // of switching off instantly
```

These are saved on the sensor itself, so you only need to call them once —
not every time your board starts up.

To check what they're currently set to (e.g. to show it in a settings
screen):

```cpp
float meters; uint16_t seconds;
radar.enableConfig();
radar.readMaxDistanceMeters(meters);
radar.readDisappearDelaySec(seconds);
radar.endConfig();
```

## Auto-calibration (let the sensor tune itself to your room)

Run this once, with nobody moving around in the room:

```cpp
radar.startCalibration();
uint8_t percent;
do {
    radar.calibrationProgress(percent);
    delay(200);
} while (percent < 100);
Serial.println("Calibration done");
```

## Everything else the sensor can do

The sensor doesn't just say "someone's there" — it measures signal strength
at 16 different distance bands ("gates"), each ~0.7m apart, and lets you set
a separate sensitivity for every one of them. This is how the Presently app's
tuning screen works, and you can do the same thing yourself.

### Seeing the raw signal at every distance

This is exactly what the Presently app's live tuning graph shows — the
actual signal, gate by gate, updating in real time, so you can see how
close a reading is to triggering before it does.

```cpp
radar.setEngineeringMode(true);     // turns on the detailed data stream

// later, in loop(), after radar.loop():
if (radar.haveEnergyGates()) {
    for (int gate = 0; gate < 16; gate++) {
        Serial.print(radar.triggerEnergyDb(gate));      // signal for "moving"
        Serial.print("/");
        Serial.print(radar.motionlessEnergyDb(gate));    // signal for "still"
        Serial.print("  ");
    }
    Serial.println();
}
```

There are two numbers per gate, because "moving" and "sitting still" are
detected separately: every gate has its own trigger reading (compared
against `setAndSaveTriggerThresholdDb`) and its own motionless reading
(compared against `setAndSaveMotionlessThresholdDb`, below). Both are in dB.
The further down the list, the further away that gate is.

### Adjusting sensitivity at a specific distance

Say gate 3 (roughly 2.1–2.8m away) is too sensitive — a curtain moving is
triggering it. Raise its threshold so only stronger signals count:

```cpp
radar.setAndSaveTriggerThresholdDb(3, 25.0);      // for movement
radar.setAndSaveMotionlessThresholdDb(3, 30.0);   // for sitting still
```

Gate numbers are 0–15. Higher number = further away. Higher dB = less
sensitive (needs a stronger signal to trigger).

### Setting every gate at once

If you've tuned all 16 gates and want to save them together (faster than 16
separate calls):

```cpp
float trigger[16]    = { /* your 16 values */ };
float motionless[16] = { /* your 16 values */ };
radar.saveAllThresholds(trigger, motionless);
```

### Reading a gate's current threshold

```cpp
float db;
radar.enableConfig();
radar.readTriggerThresholdDb(3, db);
radar.endConfig();
Serial.println(db);
```

### Checking if something's blocking the signal

```cpp
uint8_t status;
radar.readPowerInterference(status);
// 0 = hasn't been checked yet, 1 = clear, 2 = something's interfering
```

### Auto-gain (fixing a sensor that's "blinded" by something close by)

```cpp
radar.startAutoGain();
radar.autoGainDone();   // waits until the sensor finishes adjusting itself
```

### Getting the sensor's info

```cpp
String version, serial;
radar.enableConfig();
radar.readFirmwareVersion(version);
radar.readSerialNumber(serial);
radar.endConfig();
```

### Checking the sensor is actually talking to you

Useful for a device-health screen, like the one in the Presently app:

```cpp
radar.connected();      // true if it's sent data in the last 2 seconds
radar.bytesReceived();  // total bytes ever received (keeps climbing if it's alive)
radar.lastByteMs();     // millis() of the most recent byte
```

If `bytesReceived()` is climbing but `presence()`/`distanceCm()` never
change, the wiring is fine but something's wrong with reading the data. If
it stops climbing entirely, the sensor itself has gone quiet — check power
and wiring.

### Not freezing your display/animation while changing settings

Every call above that changes a setting (thresholds, max distance,
calibration, auto-gain) *waits* for the sensor to reply — up to a second or
more each. On most boards, nothing else in your sketch runs during that
wait, so a screen or animation freezes. If you're building something with a
display (again, like the Presently app's settings screen), give the library
a function to keep calling while it waits:

```cpp
void setup() {
    radar.begin(Serial1);
    radar.onIdle(myDisplay.refresh);   // called repeatedly while radar waits
}
```

See `examples/NonBlockingUI` for a working demo of this.

For the full list of every call available, open `LD2402.h` — every function
is commented with what it does right above it.

## A more advanced example

Look in the `examples/` folder for ready-to-run sketches:

- **BasicPresence** — the simple example above, with more comments
- **FullControl** — reading the sensor's raw "energy" signal at every
  distance, and adjusting sensitivity per distance band
- **NonBlockingUI** — how to change settings on the sensor without freezing
  the rest of your sketch (a display, an animation, etc.) while it does so

## Something not covered here?

This file only covers everyday use. If you want to understand *why* the
library works the way it does — quirks of the sensor itself, timing details,
raw protocol bytes — see `trash/README.old.*.md`, which has the full
engineering writeup.

## License

MIT — see [LICENSE](LICENSE).
