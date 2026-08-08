# LD2402

An easy way to use the Hi-Link HLK-LD2402 presence sensor (a small radar
module that can tell if someone is in a room) from Arduino or PlatformIO.

It tells you:
- Is someone there? (`presence`)
- Are they moving, or sitting still? (`activity`)
- How far away are they? (`distanceCm`)

Unlike a PIR sensor, it holds detection on someone who has stopped moving —
sitting, reading, sleeping — which is the whole reason to use radar here.
That needs one line of setup; see [the quick start](#the-simplest-example).

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
    switch (radar.activity()) {
        case LD2402::Absent: break;
        case LD2402::Moving:
            Serial.print("moving, ");
            Serial.print(radar.distanceCm());
            Serial.println(" cm away");
            break;
        case LD2402::Still:
            Serial.print("sitting still, ");
            Serial.print(radar.distanceCm());
            Serial.println(" cm away");
            break;
    }
}
```

Two things and you're done: `begin()` once, and `loop()` every time round
your sketch's `loop()`.

### One thing the library does for you

The sensor has two output formats, and **it starts in the wrong one:**

| | Text (the sensor's default) | Engineering |
|---|---|---|
| Someone there? | yes | yes |
| Distance | yes | yes |
| **Moving vs still** | **no** | **yes** |
| Per-gate signal levels | no | yes |

In text format a person sitting perfectly still reads as *moving* — there is
nothing in that format to say otherwise.

So `loop()` switches the sensor to engineering mode by itself, and switches
it back if the sensor restarts (that setting lives in the sensor's RAM, so a
brownout or a loose wire wipes it). You don't have to do anything.

If you specifically want the lighter text format and can live without
moving/still, turn it off with `radar.setEngineeringMode(false)`.

You do **not** need `cacheThresholds()` for still detection. The sensor makes
that decision itself. That call only avoids needless writes to the sensor's
flash — see [below](#after-calibrating-or-auto-gaining-re-read-the-thresholds).

## The main things you can ask it

| Call | What it gives you |
|---|---|
| `radar.activity()` | `LD2402::Absent` / `Moving` / `Still` — **the one to prefer** |
| `radar.presence()` | `true`/`false` — is anyone there |
| `radar.distanceCm()` | how far away, in centimetres |
| `radar.connected()` | `true` if the sensor has sent data in the last 2 seconds |
| `radar.read()` | all of the above, in one go |
| `radar.isMoving()` / `radar.isStill()` | the same answer as two booleans, if that reads better in your code |

`activity()` gives you one answer instead of three booleans you have to
combine yourself:

```cpp
switch (radar.activity()) {
    case LD2402::Absent: Serial.println("empty room");        break;
    case LD2402::Moving: Serial.println("someone's moving");   break;
    case LD2402::Still:  Serial.println("someone's sitting still"); break;
}
```

Or grab everything at once:

```cpp
LD2402::Reading r = radar.read();
if (r.presence) {
    Serial.println(r.moving ? "moving" : "still");
    Serial.println(r.distanceCm);
}
```

**If the sensor stops responding, everything reads as empty** — `presence()`
goes false and `distanceCm()` goes 0 once nothing has arrived for 2 seconds.
That's deliberate: holding the last reading forever means an unplugged sensor
still insists someone is in the room. Use `connected()` if you need to tell
"nobody's there" apart from "the sensor is gone".

## Where moving/still comes from

The sensor decides it, and this library just reports what it says. Worth
knowing because getting this wrong cost a lot of time.

Each frame's first byte is the answer: `0` nobody, `1` moving, `2` still.
That matches the vendor manual, and it is what a real module sends.

A firmware disassembly of the module once concluded that byte could only ever
be `0` or `1` — that the moving/still distinction was destroyed inside the
chip before transmission. This library believed that, and rebuilt the answer
itself by comparing per-gate signal levels against per-gate thresholds.
Logging the raw byte on real hardware (firmware v3.3.5 — the *same* version
the disassembly examined) showed `2` arriving in ordinary use. The
disassembly had missed a code path.

That derived classifier is gone. The sensor's own still detector is a
long-window filter sensitive enough to pick up breathing; nothing built from
threshold comparisons was going to match it.

**If you ever doubt this: log the byte.** Don't re-derive it from a document,
including this one.

### "Still" never triggers — what to check

1. **Engineering mode off.** Check `haveEnergyGates()`. `loop()` turns it on
   by itself, so this should only happen if you turned it off, or if the
   sensor isn't responding to config commands at all.
2. **Too close or too far.** Deep-stillness detection only covers roughly
   **2.8 m to 4.9 m** (gates 4–7). That range is hard-wired into the sensor;
   no threshold changes it. Outside it, a motionless person eventually reads
   as absent.
3. **Not actually still enough.** The sensor is sensitive — small movements
   still count as movement.

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

There are two numbers per gate, because the sensor runs two detectors: one
tuned for movement (compared against `setAndSaveTriggerThresholdDb`) and one
for stillness (compared against `setAndSaveMotionlessThresholdDb`, below).
Both are in dB. The further down the list, the further away that gate is.

These are the numbers the *sensor* compares against its thresholds to decide
moving vs still. You don't need to do that comparison yourself — the sensor
already hands you the result. They're here so you can see how close a reading
is to triggering, which is what a tuning screen needs.

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

### Restarting the sensor

```cpp
radar.reboot();
```

Handy when it's got into a strange state. It comes back in plain text mode —
that setting isn't saved on the sensor — but `loop()` notices and puts it
back within a few seconds. Thresholds *are* saved and survive the reboot.

There's no factory-reset command; the sensor doesn't have one. Restoring
defaults means writing the values back yourself.

### After calibrating or auto-gaining, re-read the thresholds

Both rewrite the sensor's thresholds, so the library's local copy of them is
now wrong. It drops that copy automatically. Detection keeps working — the
sensor classifies moving/still on its own — but until you refresh the copy,
the library can't tell whether a threshold you're setting is already the
value the sensor holds, so it writes it again regardless:

```cpp
radar.startCalibration();
// ...poll calibrationProgress() to 100...
radar.cacheThresholds();     // avoids redundant flash writes again
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

### Using it from more than one task (ESP32 / FreeRTOS)

This library isn't thread-safe. It's built for the Arduino model — one
`loop()`, one caller. If you're on an ESP32 and want to change settings from
a web handler while another task runs `radar.loop()`, put your own mutex
around it — or use the ESP-IDF version, below, which is built for that.

## The ESP-IDF version

`esp-idf/ld2402/` is the same driver as an ESP-IDF component: same protocol,
same quirks, same comments, but with its own task and mutexes instead of the
Arduino `loop()`/`onIdle()` model. Blocking config calls simply block, because
a task is allowed to.

Point `EXTRA_COMPONENT_DIRS` at `esp-idf/`, then:

```c
ld2402_config_t cfg = {};
cfg.uart_port = UART_NUM_1;
cfg.pin_tx = 21;          // ESP TX -> module R
cfg.pin_rx = 20;          // ESP RX <- module T
cfg.rx_buf_size = 8192;   // 0 = 1KB default; raise it if your board does long flash writes
cfg.event_cb = my_logger; // optional: module went quiet, came back, mode restored
ld2402_init(&cfg);
```

Calls are `ld2402_*` versions of the same things — `ld2402_get_reading()`,
`ld2402_set_and_save_max_distance_m()`, and so on. Two additions worth knowing:

- **Engineering mode is on by default and stays on**, same as the Arduino
  version — a watchdog task puts it back within ~5 s if the module reboots on
  its own, so still detection survives a module power blip untouched.
- **`ld2402_reboot()`** restarts the module itself. Handy when it wedges;
  engineering mode and the threshold cache come back on their own.
- **`reading.activity`** is `LD2402_ABSENT` / `MOVING` / `STILL` — the same
  single answer as the Arduino side's `activity()`. `presence`, `moving` and
  `still` are still in the struct (Home Assistant wants them as separate
  entities), but all four come from the module's one state byte, so they
  cannot disagree.
- **`ld2402_get_cached_*()`** return the driver's own copies of the
  thresholds, max range and disappear delay without touching the UART. The
  slow reads cost a config-mode session and a round trip per value — 32 of
  them for the full threshold set — which on a device serving a web UI is
  seconds of everything else being unanswerable. The cache is exact, not an
  approximation, and it is what lets a write skip the module's flash when the
  value asked for is already there.

### Config calls while the module is busy

`ld2402_enable_config()` — which every read and write goes through — knows
when the module is unavailable, and treats two cases differently:

- **An operation is still running** (calibration, auto-gain): returns `false`
  straight away. Calibration takes about a minute, and blocking a caller that
  long is worse than telling it the module is busy.
- **One has just finished**: waits, up to 6 seconds. The module stays
  unresponsive for a few seconds afterwards, and a write issued in that gap
  gets no ACK and looks like a failure — which is exactly how a settings
  restore run straight after a calibration used to fail.

So a caller does not need to know about any of this; it either succeeds or is
told the module is busy.

### What the event callback reports

If you set `event_cb`, expect these:

| Message | Means |
|---|---|
| `sensor module connected` | first frames seen after start-up |
| `sensor paused for calibration` / `...gain adjustment` | it stopped streaming because you asked it to |
| `sensor resumed after ...` | streaming again after one of those |
| `sensor module stopped responding` | silence nobody asked for — **this is the one worth investigating** |
| `sensor module back after Ns silent` | recovered from that |
| `engineering mode restored after module restart` | the module rebooted itself and the watchdog put it back |

The paused/resumed pair exists so a routine tune doesn't read as a fault. They
are deliberately worded differently from the unexplained ones.

Set `rx_buf_size` generously if the host writes flash while running. Those
writes run with the flash cache disabled, which halts the driver's task along
with everything else, and whatever the module sends meanwhile has to sit in
that buffer — about 775 bytes/s, so 1KB covers only ~1.3 seconds.

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
