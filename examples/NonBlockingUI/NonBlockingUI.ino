// NonBlockingUI -- keeping your sketch responsive during slow sensor calls
//
// Every configuration call in this library blocks: it sends a command and waits
// for the module to answer. On a single-threaded board (any AVR, ESP8266, or a
// single-task ESP32 sketch) nothing else in your sketch runs during that wait.
// A display freezes, an animation stops, a button stops responding.
//
// This sketch makes that visible. The built-in LED blinks once a second. Every
// 15 seconds the sketch reads all 32 gate thresholds -- about 32 round-trips to
// the module -- and prints how long it took.
//
// Press 'n' in the Serial Monitor to toggle onIdle() on and off, and watch the
// LED while the read is happening:
//
//     onIdle OFF -> the LED freezes for the whole read
//     onIdle ON  -> the LED keeps blinking normally
//
// The read takes exactly as long either way. onIdle() doesn't make the sensor
// faster; it gives your sketch the waiting time back.
//
// Wiring: sensor T -> board RX, sensor R -> board TX, plus 3.3V and GND.
// See the README for why the sensor should have a UART to itself.

#include <LD2402.h>

LD2402 radar;

// ---------------------------------------------------------------------------
// The idle hook.
//
// THE ONE RULE: this is called *very* often -- measured at roughly 8,000 times
// a second while the driver is waiting. So the first thing it does must be a
// cheap test that usually says "nothing to do yet" and returns. Anything
// expensive here, multiplied by thousands of calls, costs more than the stall
// you are trying to fix.
//
// The "is it time yet?" shape below is the usual way to write one.
//
// Also: do NOT call any radar.* method from in here. The driver is in the
// middle of a command/response exchange the entire time this runs.
// ---------------------------------------------------------------------------
void blinkLed() {
    static unsigned long last = 0;
    static bool on = false;
    if (millis() - last < 500) return;   // cheap early-out, hit ~99.99% of the time
    last = millis();
    on = !on;
    digitalWrite(LED_BUILTIN, on ? LOW : HIGH);   // most boards: LOW = lit
}

bool useIdleHook = true;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // The module's baud rate is fixed at 115200 and isn't configurable.
    Serial.begin(115200);
    radar.begin(Serial);

    // This is the whole integration: one call, once. Pass nullptr (or simply
    // never call this) to go back to the default blocking behaviour.
    radar.onIdle(blinkLed);
}

void readAllThresholds() {
    unsigned long t0 = millis();

    radar.enableConfig(2500);
    for (uint8_t gate = 0; gate < 16; gate++) {
        float motion, micro;
        radar.readTriggerThresholdDb(gate, motion);
        radar.readMotionlessThresholdDb(gate, micro);
    }
    radar.endConfig(300);

    // Printing only after the read, so the timing isn't skewed by serial I/O.
    // (In a real sketch sharing this UART with the sensor, don't print at all --
    // see the README's warning about that.)
    unsigned long took = millis() - t0;
    Serial.print(F("read 32 thresholds in "));
    Serial.print(took);
    Serial.println(F("ms"));
}

void loop() {
    radar.loop();      // keeps presence/distance up to date
    blinkLed();        // the same function, driving the LED normally

    if (Serial.available() && Serial.read() == 'n') {
        useIdleHook = !useIdleHook;
        radar.onIdle(useIdleHook ? blinkLed : nullptr);
        Serial.print(F("onIdle "));
        Serial.println(useIdleHook ? F("ON  - LED keeps blinking during reads")
                                   : F("OFF - LED will freeze during reads"));
    }

    static unsigned long lastRead = 0;
    if (millis() - lastRead > 15000) {
        lastRead = millis();
        readAllThresholds();
    }
}
