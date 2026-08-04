#include "ld2402.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "ld2402";

// Optional notification hook. The driver has a handful of things worth
// telling a user about -- the module going silent, coming back, engineering
// mode being restored after it rebooted itself -- which used to go straight
// into the host project's own event log. That was the only reason this file
// could not be lifted out and reused, so it is a callback now: set it and
// they are yours to route, leave it null and they are only ESP_LOG lines.
static ld2402_event_cb_t s_event_cb = nullptr;

static void notify(const char *fmt, ...)
{
    if (!s_event_cb) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s_event_cb(buf);
}

// Set by ld2402_init(). The Arduino version takes a Stream; here the caller
// names a UART and two pins, since ESP-IDF has no equivalent abstraction and
// hard-coding them is exactly what made this file project-specific.
static uart_port_t UART_PORT = UART_NUM_1;
static int s_pin_tx = -1, s_pin_rx = -1;

#define UART_BAUD       115200
// Default UART receive buffer. Overridable via ld2402_config_t::rx_buf_size,
// and worth overriding on any board that does long flash writes: those run
// with the flash cache disabled, halting this driver's task along with
// everything else running from flash, and whatever the module sends meanwhile
// has to sit in this buffer until the task runs again. Engineering frames are
// ~775 bytes/s, so 1KB covers only about 1.3 seconds of stall before frames
// start being lost and the reading goes stale.
#define UART_RX_BUF_DEFAULT 1024
static int s_rx_buf = UART_RX_BUF_DEFAULT;

// A frame's parsed-out reading is considered fresh for this long; past it,
// `connected` reads false even though the last value is still cached. Same
// 2000ms the original driver used, carried over unchanged.
#define STALE_US        (2000 * 1000LL)

// ---- byte/frame constants from the HLK-LD2402 manual v1.08, section 5 ----
static const uint8_t CMD_HDR[4]  = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FOOT[4] = {0x04, 0x03, 0x02, 0x01};
static const uint8_t ENG_HDR[4]  = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t ENG_FOOT[4] = {0xF8, 0xF7, 0xF6, 0xF5};

static float dbFromRaw(uint32_t raw) { return raw > 0 ? 10.0f * log10f((float)raw) : 0.0f; }
static uint32_t rawFromDb(float db) { return (uint32_t)roundf(powf(10.0f, db / 10.0f)); }

// ---------------------------------------------------------------------------
// Shared state. `s_reading` is written only by ld2402_task (streaming parse)
// and by config calls while they hold s_uart_mutex -- both cases already
// serialize against each other via that same mutex, so `s_reading_lock` only
// needs to protect readers (ld2402_get_reading()) from a torn read, not
// writers from each other.
// ---------------------------------------------------------------------------
static ld2402_reading_t s_reading;
static SemaphoreHandle_t s_reading_lock;

// Guards the UART itself, held briefly per individual command/ACK exchange.
// ld2402_task's poll loop takes it non-blocking each cycle; every raw exchange
// below takes it for that one exchange only. See the comment in radar.h for
// why this replaces the ESP8266 driver's onIdle() callback entirely.
static SemaphoreHandle_t s_uart_mutex;

// Guards a whole logical config *session* (ld2402_enable_config() through the
// matching ld2402_end_config()), which is a different, longer-lived thing than
// a single exchange. Without this, two tasks each calling a convenience
// function like ld2402_set_and_save_max_distance_m() could interleave: both
// enable/set/save/end steps take and release s_uart_mutex individually, so
// nothing stopped task B's whole session running in the gap between two of
// task A's steps, corrupting whichever one finishes "end config" first out
// from under the other. Taken by ld2402_enable_config() and released by
// ld2402_end_config(); every composite function below is already just a
// enable/.../end sequence, so this falls out for free once those two hold it.
static SemaphoreHandle_t s_session_mutex;

static void setReading(const ld2402_reading_t &r) {
    xSemaphoreTake(s_reading_lock, portMAX_DELAY);
    s_reading = r;
    xSemaphoreGive(s_reading_lock);
}

void ld2402_get_reading(ld2402_reading_t *out) {
    // Callable before ld2402_init(). "No sensor yet" is the truthful answer and
    // exactly what a disconnected sensor reports anyway, so there is nothing to
    // gain from panicking -- which is what taking a NULL mutex here did, on the
    // very first boot on hardware. See the ordering note in main.cpp.
    if (!s_reading_lock) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_reading_lock, portMAX_DELAY);
    *out = s_reading;
    xSemaphoreGive(s_reading_lock);
    out->connected = out->last_update_us != 0 &&
                      (esp_timer_get_time() - out->last_update_us) < STALE_US;
    // A disconnected sensor reports nothing, rather than its last opinion
    // forever. Held readings are worse than no readings: pull the sensor's
    // plug while someone is in the room and every consumer -- the app, MQTT,
    // Home Assistant, the LED -- would go on showing "occupied" indefinitely,
    // with only the separate `connected` flag to contradict it, and nothing
    // forcing anyone to look at that.
    if (!out->connected) {
        out->presence = false;
        out->moving = false;
        out->still = false;
        out->activity = LD2402_ABSENT;
        out->distance_cm = 0;
    }
}

// ---------------------------------------------------------------------------
// Raw UART helpers. Caller must already hold s_uart_mutex.
// ---------------------------------------------------------------------------
static bool uartReadByte(uint8_t *b, TickType_t timeout) {
    return uart_read_bytes(UART_PORT, b, 1, timeout) == 1;
}

static void uartWrite(const uint8_t *buf, size_t len) {
    uart_write_bytes(UART_PORT, (const char *)buf, len);
}

static void sendCommand(uint16_t word, const uint8_t *value, uint16_t valueLen) {
    uint16_t len = 2 + valueLen;
    uartWrite(CMD_HDR, 4);
    uint8_t hdr[4] = {(uint8_t)(len & 0xFF), (uint8_t)(len >> 8),
                       (uint8_t)(word & 0xFF), (uint8_t)(word >> 8)};
    uartWrite(hdr, 4);
    if (valueLen) uartWrite(value, valueLen);
    uartWrite(CMD_FOOT, 4);
}

// Blocks (on this calling task) until a full FD-FC-FB-FA...04-03-02-01 frame
// arrives or timeoutMs elapses. Returns the frame's word field and body
// (word+status, or word+status+extra) -- same contract as the original
// readFrameBlocking(), minus the onIdle() plumbing this concurrency model no
// longer needs.
static bool readFrameBlocking(uint16_t &word, uint8_t *body, uint16_t &bodyLen,
                               uint16_t maxBody, uint16_t timeoutMs) {
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    uint8_t match = 0;
    uint8_t b;

    auto msLeft = [&]() -> TickType_t {
        int64_t remaining = (deadline - esp_timer_get_time()) / 1000;
        return remaining > 0 ? pdMS_TO_TICKS(remaining) : 0;
    };

    while (esp_timer_get_time() < deadline) {
        if (!uartReadByte(&b, msLeft())) continue;
        if (match < 4) {
            if (b == CMD_HDR[match]) match++;
            else match = (b == CMD_HDR[0]) ? 1 : 0;
            if (match < 4) continue;
        }
        uint8_t lenBuf[2];
        for (uint8_t got = 0; got < 2;) {
            if (!uartReadByte(&lenBuf[got], msLeft())) return false;
            got++;
        }
        uint16_t len = lenBuf[0] | ((uint16_t)lenBuf[1] << 8);
        // A nonsense length means those four "header" bytes were noise that
        // happened to look like FD FC FB FA, not a real frame. Resync and keep
        // hunting with the time that's left, rather than failing the whole
        // exchange -- one corrupt byte used to abort a wait with most of its
        // budget unspent, turning a recoverable glitch into a failed command.
        if (len < 2 || len > maxBody) { match = 0; continue; }
        for (uint16_t idx = 0; idx < len;) {
            if (!uartReadByte(&body[idx], msLeft())) return false;
            idx++;
        }
        // Check the footer instead of discarding four bytes. The streaming
        // parser validates its own footer for exactly this reason: without it,
        // noise that produced a plausible header and length is accepted as a
        // genuine frame.
        bool footOk = true;
        for (uint8_t f = 0; f < 4;) {
            uint8_t got;
            if (!uartReadByte(&got, msLeft())) return false;
            if (got != CMD_FOOT[f]) footOk = false;
            f++;
        }
        if (!footOk) { match = 0; continue; }
        word = body[0] | ((uint16_t)body[1] << 8);
        bodyLen = len - 2;
        for (uint16_t i = 0; i < bodyLen; i++) body[i] = body[i + 2];
        return true;
    }
    return false;
}

static bool waitAck(uint16_t word, uint16_t timeoutMs,
                     uint8_t *extra = nullptr, uint16_t extraCap = 0, uint16_t *extraLen = nullptr) {
    uint16_t wantWord = word + 0x0100;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    static uint8_t body[200];
    while (esp_timer_get_time() < deadline) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = (uint16_t)((deadline - esp_timer_get_time()) / 1000);
        if (!readFrameBlocking(gotWord, body, bodyLen, sizeof(body), remaining)) return false;
        if (gotWord != wantWord) continue;   // stray frame, keep waiting
        if (bodyLen < 2) return false;
        uint16_t status = body[0] | ((uint16_t)body[1] << 8);
        if (status != 0) return false;
        if (extra && extraLen) {
            uint16_t n = bodyLen - 2;
            if (n > extraCap) n = extraCap;
            memcpy(extra, body + 2, n);
            *extraLen = n;
        }
        return true;
    }
    return false;
}

static bool waitEvent(uint16_t word, uint16_t timeoutMs) {
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    static uint8_t body[200];
    while (esp_timer_get_time() < deadline) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = (uint16_t)((deadline - esp_timer_get_time()) / 1000);
        if (!readFrameBlocking(gotWord, body, bodyLen, sizeof(body), remaining)) return false;
        if (gotWord == word) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Streaming parse -- runs inside ld2402_task, one byte at a time, whenever the
// UART mutex isn't held by a config call.
// ---------------------------------------------------------------------------
enum ParseState { P_IDLE, P_HDR2, P_HDR3, P_HDR4, P_LEN1, P_LEN2, P_BODY, P_FOOT1, P_FOOT2, P_FOOT3, P_FOOT4 };
static ParseState s_pstate = P_IDLE;
static uint16_t s_bodyLen = 0, s_bodyIdx = 0;
static uint8_t s_body[200];
static char s_lineBuf[40];
static uint8_t s_lineLen = 0;

// The module's `state` byte, raw. NOT an enum of none/moving/still -- that
// is what this used to be read as, and it was wrong in both directions.
//
// From the firmware reverse-engineering writeup (LD2402_firmware_RE.md
// §9, §11c), the byte is built as:
//
//     state = presence;                 // 0 or 1
//     if (<reporting sub-mode>) state += 0x10;
//
// so the only values that ever appear on the wire are 0x00, 0x01, 0x10 and
// 0x11. It never sends 2. Reading it as "1 = moving, 2 = still" therefore
// meant:
//   * 0x11 (present, sub-mode set) -> neither moving nor still, which the
//     app renders as "Still" because that's its fallback. This is the
//     "says still even when I'm moving", and the sub-mode flag toggling is
//     the "it keeps changing between moving and still".
//   * 0x10 (NOT present, sub-mode set) -> `!= 0`, so reported as presence.
//     A false-presence bug hiding behind the same mistake.
//
// The same section is explicit that the moving/still distinction does not
// survive into the module's presence output at all -- the merge is a plain
// OR and only one presence bit comes out. So it cannot be recovered from
// this byte by any masking. It has to be derived here, from the per-gate
// energies against the per-gate thresholds, which is what the engineering
// frame carries the two energy arrays for. That is also why every previous
// attempt to make moving/still respond to threshold edits failed: nothing
// was ever comparing them.
static uint8_t s_state = 0;
static uint16_t s_distanceCm = 0;
static bool s_engineering = false;
static uint32_t s_energy[32] = {0};

// Per-gate thresholds, mirrored here so publishReading() can classify
// moving vs still without a UART round trip per frame (which would mean
// entering config mode ~6 times a second and stalling the stream). Kept in
// step by ld2402_cache_thresholds(), called after any read or write of
// them. Invalid until the first read lands, and the classifier falls back
// to "presence means moving" while it is.
static float s_trigger_th[16];
static float s_motionless_th[16];
// Which gates have actually been read back, one bit each. The cache only
// counts as usable once every gate in both sets has a real value --
// classifying against a half-filled table would compare live energy to a
// zero for the gates not yet read, and every one of those would read as
// permanently over threshold.
static uint16_t s_trigger_seen = 0;
static uint16_t s_motionless_seen = 0;
static bool s_thresholds_valid = false;

// Same idea for the two module settings the app's Save button always sends
// together, changed or not. -1 means never read, so the first write always
// goes through rather than being compared against a guess.
static float s_max_distance_m = -1;
static int32_t s_disappear_delay_s = -1;

static void note_thresholds_progress(void) {
    if (s_trigger_seen == 0xFFFF && s_motionless_seen == 0xFFFF) {
        s_thresholds_valid = true;
    }
}

// Throws away the cached copy of the module's 32 thresholds.
//
// Two things depend on that cache being true, and both fail silently when it
// isn't: publishReading() classifies moving vs still by comparing live
// energies against it, and the write paths below skip a flash write when the
// value asked for matches what the cache says is already there. So a stale
// cache means a misclassified room *and* threshold writes that get dropped as
// redundant when they aren't.
//
// Calibration and auto-gain both make it stale -- see their call sites.
static void invalidate_threshold_cache(void) {
    s_thresholds_valid = false;
    s_trigger_seen = 0;
    s_motionless_seen = 0;
}

// True between "calibration started" and the poll that sees it hit 100%.
// Only threshold_cache_task() reads it, to keep itself out of config mode
// while the module is busy measuring the room.
static volatile bool s_calibrating = false;

// Set for the duration of an auto-gain run. Same purpose as s_calibrating:
// the module goes quiet while it recalibrates its front-end, and that is
// expected rather than a fault.
static volatile bool s_autogain_running = false;

// How long past the end of such an operation the silence is still expected.
//
// Clearing the flag the moment the operation returns was not enough: the
// module does not resume streaming instantly, so the watchdog caught the tail
// of the quiet period and reported exactly the dropout this was meant to
// suppress -- just a few seconds later. Measured gap was ~5s, so this leaves
// room without being long enough to hide a real fault.
static volatile int64_t s_quiet_until_us = 0;

static void expect_quiet_for(int64_t seconds) {
    s_quiet_until_us = esp_timer_get_time() + seconds * 1000000LL;
}

// Cached copies of what the module holds, readable without touching the UART.
//
// Every read below otherwise costs a config-mode session and a round trip per
// value -- 32 of them for the full threshold set, which is seconds of the
// caller's time and, on a single-task HTTP server, seconds during which
// nothing else is answered. The driver already keeps these in step (it has
// to: the classifier compares live energy against the thresholds on every
// frame), so serving a settings screen from them is free and exact.
//
// Each returns false when the value has not been learned yet, which is the
// caller's cue to do the slow read once.

bool ld2402_get_cached_thresholds(float trigger_db[16], float motionless_db[16]) {
    if (!s_thresholds_valid) return false;
    if (trigger_db) memcpy(trigger_db, s_trigger_th, sizeof(s_trigger_th));
    if (motionless_db) memcpy(motionless_db, s_motionless_th, sizeof(s_motionless_th));
    return true;
}

bool ld2402_get_cached_max_distance_m(float *meters) {
    if (s_max_distance_m < 0) return false;
    if (meters) *meters = s_max_distance_m;
    return true;
}

bool ld2402_get_cached_disappear_delay_s(uint16_t *seconds) {
    if (s_disappear_delay_s < 0) return false;
    if (seconds) *seconds = (uint16_t)s_disappear_delay_s;
    return true;
}


// What engineering mode *should* be, as last requested. The watchdog restores
// it after the module reboots itself. Defaults to on: streaming the energy
// gates is the reason to use this driver over the module's plain IO pin.
static volatile bool s_want_engineering = true;
static uint32_t s_byteCount = 0;
static int64_t s_lastByteUs = 0;
static int64_t s_lastUpdateUs = 0;

static void publishReading() {
    ld2402_reading_t r;
    // Low nibble only -- the 0x10 bit is the reporting sub-mode flag, not
    // part of the presence value. See s_state's comment.
    r.presence = (s_state & 0x0F) != 0;
    r.connected = true;   // ld2402_get_reading() recomputes this from staleness
    r.distance_cm = s_distanceCm;
    r.engineering = s_engineering;
    for (uint8_t i = 0; i < 16; i++) {
        r.trigger_db[i] = s_engineering ? dbFromRaw(s_energy[i]) : NAN;
        r.motionless_db[i] = s_engineering ? dbFromRaw(s_energy[16 + i]) : NAN;
    }

    // Moving vs still, derived here because the module doesn't report it.
    //
    // Mirrors the module's own peak selection (RE writeup §8): scan gates
    // 1..15 and take any gate whose energy clears that gate's threshold.
    // Gate 0 is skipped for the same reason the module skips it -- its
    // threshold is never evaluated, so including it would let near-field
    // clutter set a flag no configuration change could ever clear.
    //
    // Both can be true at once: they are two filter chains over the same
    // signal (fast clutter filter + 4-frame integration vs slow filter +
    // 16-frame), not mutually exclusive states, so a real person usually
    // lights up both. Callers that want one word for the UI should prefer
    // `moving` -- see the note in radar.h.
    // Presence is decided first, and absolutely: with nobody detected,
    // "moving" is not a question worth asking. The gate scan used to run
    // regardless, so a stray gate over threshold with no presence produced
    // moving=true, presence=false, still=false -- a combination that cannot
    // be true, and one no consumer of this struct could render sensibly.
    r.moving = false;
    if (!r.presence) {
        // nothing to classify
    } else if (s_engineering && s_thresholds_valid) {
        for (uint8_t g = 1; g < 16; g++) {
            if (!isnan(r.trigger_db[g]) && r.trigger_db[g] > s_trigger_th[g]) {
                r.moving = true;
                break;
            }
        }
    } else {
        // No engineering frame (module in ASCII mode) or thresholds not
        // read back yet: presence is all we have, so report it as movement
        // rather than inventing a classification. Guessing "still" here is
        // what made a walking person read as stationary.
        r.moving = true;
    }

    // Still is the complement of moving, not its own threshold test.
    //
    // Testing the motionless energies separately looks more principled and
    // isn't: the two chains are the same signal under different filters, so
    // both can clear their thresholds, neither can, or either alone can --
    // giving four combinations for what is really a two-way question, and
    // two of them ("present but neither" and "present and both") have no
    // sensible thing to display. "Present but neither" is what a person
    // sitting quietly actually produced here, and the app rendered it as
    // "still" through a fallback anyway, which is the right answer arrived
    // at by accident. Deriving it makes every state well-defined: something
    // is there, and it is either moving or it isn't.
    r.still = r.presence && !r.moving;
    // One value carrying the same decision, so callers that want a single
    // answer do not have to recombine the booleans and risk inventing a
    // fourth state that cannot occur.
    r.activity = !r.presence ? LD2402_ABSENT : (r.moving ? LD2402_MOVING : LD2402_STILL);
    r.bytes_received = s_byteCount;
    r.last_byte_us = s_lastByteUs;
    r.last_update_us = s_lastUpdateUs;
    setReading(r);
}

static void handleTextLine(void) {
    // Trim trailing \r/space already excluded by feedByte(); just null-terminate.
    if (s_lineLen == 0) return;
    if (strcmp(s_lineBuf, "OFF") == 0) {
        s_state = 0;
        s_distanceCm = 0;
        s_engineering = false;
        s_lastUpdateUs = esp_timer_get_time();
        publishReading();
        return;
    }
    const char *colon = strchr(s_lineBuf, ':');
    if (strncmp(s_lineBuf, "distance", 8) == 0 && colon) {
        s_state = 1;
        s_distanceCm = (uint16_t)atoi(colon + 1);
        s_engineering = false;
        s_lastUpdateUs = esp_timer_get_time();
        publishReading();
    }
}

static void handleTextByte(uint8_t b) {
    if (b == '\n') {
        s_lineBuf[s_lineLen] = '\0';
        handleTextLine();
        s_lineLen = 0;
        return;
    }
    if (b == '\r') return;
    if (s_lineLen < sizeof(s_lineBuf) - 1) s_lineBuf[s_lineLen++] = (char)b;
    else s_lineLen = 0;   // garbage/overlong line, drop it
}

static void handleEngineeringFrame(const uint8_t *body, uint16_t len) {
    if (len < 3) return;
    s_state = body[0];
    s_distanceCm = body[1] | ((uint16_t)body[2] << 8);
    s_engineering = true;
    if (len >= 3 + 32 * 4) {
        for (uint8_t i = 0; i < 32; i++) {
            uint16_t off = 3 + i * 4;
            s_energy[i] = (uint32_t)body[off] | ((uint32_t)body[off + 1] << 8) |
                          ((uint32_t)body[off + 2] << 16) | ((uint32_t)body[off + 3] << 24);
        }
    }
    s_lastUpdateUs = esp_timer_get_time();
    publishReading();
}

// Same header/length/footer state machine as the original feedByte(), with
// the footer validated byte-by-byte rather than just counted -- an
// unvalidated footer let electrical noise that happened to produce a
// plausible header+length get accepted as a frame, occasionally reporting
// fake presence. Carried over unchanged.
static void feedByte(uint8_t b) {
    s_byteCount++;
    s_lastByteUs = esp_timer_get_time();
    switch (s_pstate) {
        case P_IDLE:
            if (b == ENG_HDR[0]) s_pstate = P_HDR2;
            else handleTextByte(b);
            return;
        case P_HDR2: s_pstate = (b == ENG_HDR[1]) ? P_HDR3 : P_IDLE; return;
        case P_HDR3: s_pstate = (b == ENG_HDR[2]) ? P_HDR4 : P_IDLE; return;
        case P_HDR4: s_pstate = (b == ENG_HDR[3]) ? P_LEN1 : P_IDLE; return;
        case P_LEN1: s_bodyLen = b; s_pstate = P_LEN2; return;
        case P_LEN2:
            s_bodyLen |= ((uint16_t)b << 8);
            s_bodyIdx = 0;
            if (s_bodyLen == 0 || s_bodyLen > sizeof(s_body)) { s_pstate = P_IDLE; return; }
            s_pstate = P_BODY;
            return;
        case P_BODY:
            s_body[s_bodyIdx++] = b;
            if (s_bodyIdx >= s_bodyLen) s_pstate = P_FOOT1;
            return;
        case P_FOOT1: s_pstate = (b == ENG_FOOT[0]) ? P_FOOT2 : P_IDLE; return;
        case P_FOOT2: s_pstate = (b == ENG_FOOT[1]) ? P_FOOT3 : P_IDLE; return;
        case P_FOOT3: s_pstate = (b == ENG_FOOT[2]) ? P_FOOT4 : P_IDLE; return;
        case P_FOOT4:
            s_pstate = P_IDLE;
            if (b == ENG_FOOT[3]) handleEngineeringFrame(s_body, s_bodyLen);
            return;
    }
}

static void ld2402_task(void *arg) {
    uint8_t b;
    while (true) {
        // Non-blocking: if a config call holds the mutex, back off briefly and
        // retry rather than stalling behind it -- config sessions run to
        // several seconds and this task has nothing useful to do meanwhile.
        if (xSemaphoreTake(s_uart_mutex, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // Short timeout: drain whatever's waiting, then release the mutex so a
        // config call queued behind it isn't kept waiting indefinitely.
        while (uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(5)) == 1) {
            feedByte(b);
        }
        xSemaphoreGive(s_uart_mutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Restores engineering mode after the *module* restarts.
//
// Output mode lives in the LD2402's RAM, not its flash -- unlike the
// thresholds, which do survive (verified by pulling the sensor's power and
// reading all 32 back unchanged). So a power blip on the module alone, with
// the ESP32 still running, drops it back to plain text output: the energy
// arrays stop, moving/still falls back to "presence means moving", and the
// tuning screen shows its "engineering data is off" banner with no
// explanation of what happened. It had to be turned back on by hand.
//
// The desired state is whatever was last asked for through
// ld2402_set_engineering_mode(); nothing was enforcing it. This watches for
// the module reporting in text mode while it should be in engineering mode,
// and re-applies.
//
// Deliberately unhurried: the module has to be actually alive and settled
// before it will accept a config session, and a reboot takes a few seconds.
// Re-checking every 5s is far below the "user notices" threshold and costs
// nothing while the mode already matches.
// Also the one place that notices the module coming and going, reported
// through the event callback. Worth surfacing: a module that quietly reboots,
// drops out of engineering mode and goes silent looks, from the application
// side, exactly like the application breaking.
static void engineering_watchdog_task(void *arg) {
    (void)arg;
    bool was_connected = false;
    bool first_pass = true;
    int64_t down_since_us = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        ld2402_reading_t r;
        ld2402_get_reading(&r);

        // ── module presence on the UART ──
        //
        // Silence we asked for is not silence worth reporting. Auto-gain and
        // calibration both stop the module streaming while they run -- long
        // enough to trip the 2s freshness window -- so without this, every
        // routine tune left "sensor module stopped responding ... back after
        // 5s silent" in the log. That reads as a fault, and sends whoever
        // finds it later hunting for one.
        //
        // The state is still tracked, so a genuine dropout that happens to
        // start during an operation is still noticed once it ends.
        const bool expected_quiet = s_calibrating || s_autogain_running ||
                                    esp_timer_get_time() < s_quiet_until_us;
        if (r.connected != was_connected && !expected_quiet) {
            if (r.connected) {
                if (first_pass) {
                    notify("sensor module connected");
                } else {
                    notify("sensor module back after %llds silent",
                                       (esp_timer_get_time() - down_since_us) / 1000000);
                }
            } else {
                down_since_us = esp_timer_get_time();
                notify("sensor module stopped responding");
            }
            was_connected = r.connected;
            first_pass = false;
        }

        if (!s_want_engineering) continue;

        // Only act when the module is demonstrably alive and demonstrably
        // not in engineering mode. `connected` false means no frames at all
        // -- it may still be booting, and shouting config at it then just
        // wastes a session that will time out.
        if (!r.connected || r.engineering) continue;

        ESP_LOGW(TAG, "module is in text mode but engineering is enabled -- restoring");
        // Worth a log line rather than silent self-healing: if this starts
        // firing repeatedly it means the module is rebooting over and over,
        // which is a power or wiring problem the log should make obvious
        // instead of hiding behind a feature that appears to work.
        if (ld2402_set_engineering_mode(true, 2500)) {
            notify("engineering mode restored after module restart");
            // Thresholds live in module flash and survive its reboot, so the
            // cache is still correct -- no need to re-read all 32 here.
        } else {
            notify("could not restore engineering mode");
        }
    }
}

// Keeps the classifier's threshold cache primed: reads all 32 thresholds a
// few seconds after boot, and again any time something invalidates them.
//
// This used to run once and delete itself, which was fine while nothing ever
// dropped the cache. Now that calibration and auto-gain do -- they have to,
// since both make the cached numbers wrong -- a one-shot task would leave
// moving/still degraded from the first calibration until the next reboot.
// So it stays resident and re-primes instead, which costs one sleeping task
// and removes a whole class of "works until you calibrate" behaviour.
//
// It deliberately does nothing while a calibration is running: priming means
// entering config mode, and the module is busy measuring the room.
static void threshold_cache_task(void *arg) {
    (void)arg;
    bool announced = false;
    int attempt = 0;
    while (true) {
        if (s_thresholds_valid || s_calibrating) {
            attempt = 0;
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        // First pass after boot gets a short wait (the module needs a moment
        // after power-up before it accepts config mode); repeated failures
        // back off so a module that never answers isn't hammered forever.
        vTaskDelay(pdMS_TO_TICKS(attempt == 0 ? 4000 : 10000));
        if (s_calibrating) continue;
        attempt++;
        if (!ld2402_enable_config(2500)) {
            if (attempt == 5 && !announced) {
                announced = true;
                ESP_LOGW(TAG, "threshold cache unavailable -- moving/still degraded");
                notify("could not read gate thresholds -- motion/still unreliable");
            }
            continue;
        }
        float db;
        for (int g = 0; g < 16; g++) {
            ld2402_read_trigger_threshold_db(g, &db, 1000);
            ld2402_read_motionless_threshold_db(g, &db, 1000);
        }
        ld2402_end_config(1000);
        if (s_thresholds_valid) {
            ESP_LOGI(TAG, "threshold cache primed");
            announced = false;   // arm the warning again for the next re-prime
            attempt = 0;
        }
    }
}

esp_err_t ld2402_init(const ld2402_config_t *cfg) {
    if (!cfg || cfg->pin_tx < 0 || cfg->pin_rx < 0) return ESP_ERR_INVALID_ARG;
    UART_PORT   = cfg->uart_port;
    s_pin_tx    = cfg->pin_tx;
    s_pin_rx    = cfg->pin_rx;
    s_event_cb  = cfg->event_cb;
    if (cfg->rx_buf_size > 0) s_rx_buf = cfg->rx_buf_size;

    s_reading_lock = xSemaphoreCreateMutex();
    s_uart_mutex = xSemaphoreCreateMutex();
    s_session_mutex = xSemaphoreCreateMutex();
    if (!s_reading_lock || !s_uart_mutex || !s_session_mutex) return ESP_ERR_NO_MEM;
    memset(&s_reading, 0, sizeof(s_reading));

    uart_config_t uc = {};
    uc.baud_rate = UART_BAUD;
    uc.data_bits = UART_DATA_8_BITS;
    uc.parity = UART_PARITY_DISABLE;
    uc.stop_bits = UART_STOP_BITS_1;
    uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, s_rx_buf, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uc));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, s_pin_tx, s_pin_rx,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(ld2402_task, "radar", 4096, nullptr, 5, nullptr);
    // One-shot: fill the threshold cache so moving/still classification works
    // from boot rather than only after someone opens the tuning screen. Its
    // own task because it enters config mode, which blocks the stream for a
    // couple of seconds -- not something to do inline on the boot path.
    xTaskCreate(threshold_cache_task, "ld2402_th", 3072, nullptr, 3, nullptr);
    xTaskCreate(engineering_watchdog_task, "ld2402_eng", 3072, nullptr, 3, nullptr);
    ESP_LOGI(TAG, "started on UART%d (tx=%d rx=%d)", (int)UART_PORT, s_pin_tx, s_pin_rx);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Config / calibration. Every one of these takes s_uart_mutex for its whole
// session, pausing the streaming parse for its duration -- see radar.h.
// ---------------------------------------------------------------------------

// RAII-ish: takes the mutex in the constructor, always released in the
// destructor -- so an early `return false` mid-function (there are many
// below, ported straight from the original's early-return style) can't leak
// the lock and starve ld2402_task forever.
struct UartSession {
    bool held;
    UartSession() : held(xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {}
    ~UartSession() { if (held) xSemaphoreGive(s_uart_mutex); }
};

bool ld2402_enable_config(uint16_t timeoutMs) {
    // Wait out a known-quiet stretch before even trying.
    //
    // Calibration and auto-gain leave the module unresponsive for a few
    // seconds after they finish. A write issued in that window gets no ACK
    // and is reported as a failure -- which is exactly what happened to a
    // backup restored straight after a calibration run: "thresholds save
    // FAILED (sensor did not ack)", with the module perfectly healthy and
    // simply not listening yet.
    //
    // Deliberately before the session mutex is taken, not after. Waiting
    // while holding it would block the very operation whose quiet period we
    // are waiting on from ever finishing.
    //
    // Two cases, treated differently on purpose:
    //
    //   still running  -> refuse immediately. Calibration takes a minute, and
    //                     blocking a web request for that long is the problem
    //                     this driver spent a lot of effort removing. "Busy"
    //                     is a truthful answer the caller can act on.
    //   just finished  -> wait, but briefly. The gap is a few seconds and the
    //                     caller almost certainly wants the write to land
    //                     rather than to be told to try again.
    if (s_calibrating || s_autogain_running) return false;

    const int64_t wait_until =
        (s_quiet_until_us < esp_timer_get_time() + 6000000LL) ? s_quiet_until_us
                                                              : esp_timer_get_time() + 6000000LL;
    while (esp_timer_get_time() < wait_until && !s_calibrating && !s_autogain_running) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Session lock taken here, released by ld2402_end_config() -- see the
    // s_session_mutex comment above. If the handshake below never succeeds,
    // release it before returning; nothing will call end_config to do it for
    // us, and leaking it would starve every future config call forever.
    if (xSemaphoreTake(s_session_mutex, pdMS_TO_TICKS((uint32_t)timeoutMs + 1000)) != pdTRUE) return false;

    UartSession s;
    if (!s.held) { xSemaphoreGive(s_session_mutex); return false; }

    // Deadline-based hammering, not a fixed handful of tries -- see the long
    // comment in the original driver (LD2402.cpp) for why: breaking into
    // config mode is easy when idle or text-streaming, hard while engineering
    // frames are firehosing (~128 bytes every 165ms), and the module stops
    // streaming the instant it does accept the command, so the very next
    // attempt sees a clean ACK. Keep re-sending until one lands or time runs out.
    const uint8_t val[2] = {0x01, 0x00};
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    do {
        uint8_t discard;
        while (uartReadByte(&discard, 0)) {}   // drop buffered stream bytes
        sendCommand(0x00FF, val, 2);
        if (waitAck(0x00FF, 250)) return true;   // session_mutex stays held until end_config()
    } while (esp_timer_get_time() < deadline);

    xSemaphoreGive(s_session_mutex);
    return false;
}

bool ld2402_end_config(uint16_t timeoutMs) {
    // Pairs with ld2402_enable_config() -- always releases s_session_mutex on
    // the way out, whether or not the exit handshake itself succeeds, since
    // the module either way is no longer this call's to hold exclusively.
    UartSession s;
    bool ok = false;
    if (s.held) {
        for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
            uint8_t discard;
            while (uartReadByte(&discard, 0)) {}
            sendCommand(0x00FE, nullptr, 0);
            if (waitAck(0x00FE, timeoutMs)) ok = true;
            else vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    xSemaphoreGive(s_session_mutex);
    return ok;
}

bool ld2402_read_firmware_version(char *buf, size_t cap, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held || cap == 0) return false;
    sendCommand(0x0000, nullptr, 0);
    uint8_t extra[64];
    uint16_t n = 0;
    if (!waitAck(0x0000, timeoutMs, extra, sizeof(extra), &n) || n < 2) return false;
    uint16_t verLen = extra[0] | ((uint16_t)extra[1] << 8);
    if (verLen > n - 2) verLen = n - 2;
    if (verLen > cap - 1) verLen = cap - 1;
    memcpy(buf, extra + 2, verLen);
    buf[verLen] = '\0';
    return true;
}

bool ld2402_read_serial_number(char *buf, size_t cap, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held || cap == 0) return false;
    sendCommand(0x0011, nullptr, 0);
    uint8_t extra[40];
    uint16_t n = 0;
    if (!waitAck(0x0011, timeoutMs, extra, sizeof(extra), &n) || n < 2) return false;
    uint16_t snLen = extra[0] | ((uint16_t)extra[1] << 8);
    if (snLen > n - 2) snLen = n - 2;
    if (snLen > cap - 1) snLen = cap - 1;
    memcpy(buf, extra + 2, snLen);
    buf[snLen] = '\0';
    return true;
}

bool ld2402_set_output_mode(bool engineering, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    uint32_t mode = engineering ? 0x00000004 : 0x00000064;
    // Value field is SIX bytes: 2-byte command value (always 0) + 4-byte mode
    // (manual 5.2.8). Sending only 4 bytes made a malformed frame that left
    // the module silent after the switch -- carried over from the original.
    uint8_t val[6] = {0x00, 0x00,
                       (uint8_t)mode, (uint8_t)(mode >> 8),
                       (uint8_t)(mode >> 16), (uint8_t)(mode >> 24)};
    sendCommand(0x0012, val, 6);
    return waitAck(0x0012, timeoutMs);
}

bool ld2402_set_engineering_mode(bool on, uint16_t configTimeoutMs) {
    // Remembered so the watchdog can put the module back into this mode after
    // it reboots on its own -- output mode lives in the module's RAM, so a
    // power blip on the sensor alone silently drops it back to text.
    s_want_engineering = on;
    // Deliberately not wrapped in its own UartSession -- enable/set/end below
    // each take and release the mutex themselves, in sequence, which is
    // equivalent and avoids a nested-mutex-take deadlock.
    ld2402_enable_config(configTimeoutMs);
    bool ok = ld2402_set_output_mode(on, 1000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_read_parameter_raw(uint16_t id, uint32_t *value, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    uint8_t val[2] = {(uint8_t)(id & 0xFF), (uint8_t)(id >> 8)};
    sendCommand(0x0008, val, 2);
    uint8_t extra[4];
    uint16_t n = 0;
    if (!waitAck(0x0008, timeoutMs, extra, sizeof(extra), &n) || n < 4) return false;
    *value = (uint32_t)extra[0] | ((uint32_t)extra[1] << 8) | ((uint32_t)extra[2] << 16) | ((uint32_t)extra[3] << 24);
    return true;
}

bool ld2402_set_parameter_raw(uint16_t id, uint32_t value, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    uint8_t val[6] = {
        (uint8_t)(id & 0xFF), (uint8_t)(id >> 8),
        (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    sendCommand(0x0007, val, 6);
    return waitAck(0x0007, timeoutMs);
}

bool ld2402_set_max_distance_m(float meters, uint16_t timeoutMs) {
    int v = (int)roundf(meters * 10.0f);
    if (v < 7) v = 7;
    if (v > 100) v = 100;
    if (!ld2402_set_parameter_raw(0x0001, (uint32_t)v, timeoutMs)) return false;
    s_max_distance_m = v / 10.0f;
    return true;
}
bool ld2402_read_max_distance_m(float *meters, uint16_t timeoutMs) {
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0001, &raw, timeoutMs)) return false;
    *meters = raw / 10.0f;
    s_max_distance_m = *meters;
    return true;
}

bool ld2402_set_disappear_delay_s(uint16_t seconds, uint16_t timeoutMs) {
    if (!ld2402_set_parameter_raw(0x0004, seconds, timeoutMs)) return false;
    s_disappear_delay_s = (int32_t)seconds;
    return true;
}
bool ld2402_read_disappear_delay_s(uint16_t *seconds, uint16_t timeoutMs) {
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0004, &raw, timeoutMs)) return false;
    *seconds = (uint16_t)raw;
    s_disappear_delay_s = (int32_t)*seconds;
    return true;
}

// Send the explicit save command (0x00FD) before leaving config mode.
//
// This used to rely on exiting config mode committing everything by itself.
// It doesn't -- verified the hard way: max range and disappear delay set
// through here read back correctly, then reverted to their previous values
// after the module was power-cycled. (Per-gate thresholds happened to
// survive the same test, which is what made the old assumption look right;
// whatever commits those is not this path.)
//
// 0x00FD only *requests* a save -- the module's main loop does the erase
// and program some time afterwards -- so ld2402_save_parameters() waits
// before returning rather than letting end_config race the write.
bool ld2402_set_and_save_max_distance_m(float meters, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    // Already this value: no session, no write, no flash erase. The app's
    // Save button sends range and delay together every time it is pressed,
    // so without this, saving a delay change also rewrote an unchanged range.
    if (s_max_distance_m >= 0 && fabsf(s_max_distance_m - meters) < 0.05f) return true;
    ld2402_enable_config(configTimeoutMs);
    bool ok = ld2402_set_max_distance_m(meters, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_set_and_save_disappear_delay_s(uint16_t seconds, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (s_disappear_delay_s >= 0 && s_disappear_delay_s == (int32_t)seconds) return true;
    ld2402_enable_config(configTimeoutMs);
    bool ok = ld2402_set_disappear_delay_s(seconds, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_set_trigger_threshold_db(uint8_t gate, float db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    if (!ld2402_set_parameter_raw(0x0010 + gate, rawFromDb(db), timeoutMs)) return false;
    // Mirror into the classifier's cache. Doing it in the primitives means
    // every path -- single gate write, bulk save, calibration read-back --
    // keeps the cache honest without each caller remembering to.
    s_trigger_th[gate] = db;
    return true;
}
bool ld2402_read_trigger_threshold_db(uint8_t gate, float *db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0010 + gate, &raw, timeoutMs)) return false;
    *db = dbFromRaw(raw);
    s_trigger_th[gate] = *db;
    s_trigger_seen |= (uint16_t)1u << gate;
    note_thresholds_progress();
    return true;
}
bool ld2402_set_motionless_threshold_db(uint8_t gate, float db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    if (!ld2402_set_parameter_raw(0x0030 + gate, rawFromDb(db), timeoutMs)) return false;
    s_motionless_th[gate] = db;
    return true;
}
bool ld2402_read_motionless_threshold_db(uint8_t gate, float *db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0030 + gate, &raw, timeoutMs)) return false;
    *db = dbFromRaw(raw);
    s_motionless_th[gate] = *db;
    s_motionless_seen |= (uint16_t)1u << gate;
    note_thresholds_progress();
    return true;
}

bool ld2402_set_and_save_trigger_threshold_db(uint8_t gate, float db, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (gate > 15) return false;
    if (s_thresholds_valid && fabsf(s_trigger_th[gate] - db) < 0.05f) return true;
    ld2402_enable_config(configTimeoutMs);
    bool ok = ld2402_set_trigger_threshold_db(gate, db, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_set_and_save_motionless_threshold_db(uint8_t gate, float db, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (gate > 15) return false;
    if (s_thresholds_valid && fabsf(s_motionless_th[gate] - db) < 0.05f) return true;
    ld2402_enable_config(configTimeoutMs);
    bool ok = ld2402_set_motionless_threshold_db(gate, db, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

// Writes only the gates whose value actually changed, and skips the flash
// commit entirely when none did.
//
// The app's Save button sends all 32 thresholds every time it is pressed,
// whether one gate moved or none did. Writing all 32 and committing was 32
// parameter writes plus a flash erase on the module for what is often a
// no-op -- and the LD2402's flash is a smaller, less protected thing than
// the ESP's NVS, with no wear levelling anyone has documented. Tapping Save
// twice in a row should not cost twice.
//
// Uses the same cache the moving/still classifier keeps in step, so this
// costs a float comparison per gate. The cache is only trusted once every
// gate has been read back at least once; before that, write everything,
// because "unknown" must never be mistaken for "unchanged".
bool ld2402_save_all_thresholds(const float triggerDb[16], const float motionlessDb[16],
                                uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    ld2402_enable_config(configTimeoutMs);
    bool ok = true;
    int written = 0;

    // 0.05dB: the module stores linear power and round-trips through a log,
    // so a value read back is never bit-identical to the one written. Any
    // real edit is far larger than this.
    auto same = [](float a, float b) { return fabsf(a - b) < 0.05f; };

    if (triggerDb) {
        for (uint8_t i = 0; i < 16; i++) {
            if (s_thresholds_valid && same(triggerDb[i], s_trigger_th[i])) continue;
            ok &= ld2402_set_trigger_threshold_db(i, triggerDb[i], 1000);
            written++;
        }
    }
    if (motionlessDb) {
        for (uint8_t i = 0; i < 16; i++) {
            if (s_thresholds_valid && same(motionlessDb[i], s_motionless_th[i])) continue;
            ok &= ld2402_set_motionless_threshold_db(i, motionlessDb[i], 1000);
            written++;
        }
    }

    if (written == 0) {
        ESP_LOGI(TAG, "thresholds unchanged -- no write");
    } else if (ok) {
        ESP_LOGI(TAG, "wrote %d of 32 thresholds", written);
        ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    }
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_read_power_interference(uint8_t *status, uint16_t timeoutMs) {
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0005, &raw, timeoutMs)) return false;
    *status = (uint8_t)raw;
    return true;
}

bool ld2402_start_calibration(uint8_t triggerFactor, uint8_t holdFactor, uint8_t microFactor, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    uint16_t trig = (uint16_t)triggerFactor * 10, hold = (uint16_t)holdFactor * 10, micro = (uint16_t)microFactor * 10;
    uint8_t val[6] = {
        (uint8_t)trig, (uint8_t)(trig >> 8),
        (uint8_t)hold, (uint8_t)(hold >> 8),
        (uint8_t)micro, (uint8_t)(micro >> 8)};
    sendCommand(0x0009, val, 6);
    if (!waitAck(0x0009, timeoutMs)) return false;
    // Calibration rewrites all 32 thresholds inside the module, so every
    // cached copy here is now wrong. Nothing used to drop them, which left
    // moving/still classified against pre-calibration numbers indefinitely,
    // and left the no-op guards on the write paths comparing against values
    // the module no longer holds -- silently skipping real writes.
    invalidate_threshold_cache();
    s_calibrating = true;
    return true;
}

bool ld2402_calibration_progress(uint8_t *percent, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    sendCommand(0x000A, nullptr, 0);
    uint8_t extra[2];
    uint16_t n = 0;
    if (!waitAck(0x000A, timeoutMs, extra, sizeof(extra), &n) || n < 2) return false;
    *percent = extra[0];   // fits in one byte (0-100); extra[1] is always 0
    // Releases threshold_cache_task to go and re-read the thresholds the
    // calibration just rewrote. Polling this to completion is what every
    // caller already does, so there is no separate "calibration finished"
    // event to hang this off.
    if (*percent >= 100) {
        s_calibrating = false;
        expect_quiet_for(10);
    }
    return true;
}

bool ld2402_read_calibration_interference(bool *hadInterference, uint16_t *gateMask, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    sendCommand(0x0014, nullptr, 0);
    const uint16_t wantWord = 0x0014 + 0x0100;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    static uint8_t body[200];
    while (esp_timer_get_time() < deadline) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = (uint16_t)((deadline - esp_timer_get_time()) / 1000);
        if (!readFrameBlocking(gotWord, body, bodyLen, sizeof(body), remaining)) return false;
        if (gotWord != wantWord) continue;   // stray frame, keep waiting
        if (bodyLen < 4) return false;
        uint16_t status = body[0] | ((uint16_t)body[1] << 8);
        *gateMask = body[2] | ((uint16_t)body[3] << 8);
        *hadInterference = (status != 0);
        return true;
    }
    return false;
}

bool ld2402_save_parameters(uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    sendCommand(0x00FD, nullptr, 0);
    if (!waitAck(0x00FD, timeoutMs)) return false;
    vTaskDelay(pdMS_TO_TICKS(500));   // module needs time to commit to flash before config exits
    return true;
}

// Restarts the module (command 0x00EF, confirmed in the v3.3.5 disassembly).
// It exits config mode on its own, so nothing needs wrapping around it.
//
// Everything cached here describes the module that just went away: its
// thresholds are re-read from flash on boot, and its output mode is not
// persistent at all, so it comes back in text mode. The watchdog puts
// engineering mode back within a few seconds; the cache task re-primes the
// thresholds. Nothing else has to be done by the caller.
bool ld2402_reboot(uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    sendCommand(0x00EF, nullptr, 0);
    if (!waitAck(0x00EF, timeoutMs)) return false;
    invalidate_threshold_cache();
    s_engineering = false;
    s_state = 0;
    s_distanceCm = 0;
    notify("sensor module restart requested");
    return true;
}

bool ld2402_start_auto_gain(uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    sendCommand(0x00EE, nullptr, 0);
    if (!waitAck(0x00EE, timeoutMs)) return false;
    s_autogain_running = true;
    // Auto-gain changes the front-end gain, so the energies the module
    // reports afterwards sit on a different scale than the cached thresholds
    // were read against. Same stale-comparison problem as calibration.
    invalidate_threshold_cache();
    return true;
}

bool ld2402_auto_gain_done(uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return false;
    // The module pushes this unprompted (word 0x00F0, not a +0x0100 ACK) once
    // auto-gain finishes -- not a reply to a request we send.
    const bool done = waitEvent(0x00F0, timeoutMs);
    s_autogain_running = false;
    expect_quiet_for(10);
    return done;
}
