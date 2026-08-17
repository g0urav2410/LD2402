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

// Why the last failed call failed. See the note above ld2402_err_t in the
// header for why this is errno-style rather than a changed return type.
//
// Written only on a failure path and never cleared, so it is meaningful only
// straight after a call returned false. Not mutex-guarded: config calls are
// already serialised by s_session_mutex, and a plain enum store is atomic on
// this target, so a lock here would protect nothing that isn't protected.
static ld2402_err_t s_last_err = LD2402_OK;

static bool fail(ld2402_err_t err) { s_last_err = err; return false; }

ld2402_err_t ld2402_last_error(void) { return s_last_err; }

const char *ld2402_err_str(ld2402_err_t err) {
    switch (err) {
        case LD2402_OK:                return "ok";
        case LD2402_ERR_BUSY:          return "busy";
        case LD2402_ERR_TIMEOUT:       return "timeout";
        case LD2402_ERR_REFUSED:       return "refused";
        case LD2402_ERR_BAD_REPLY:     return "bad_reply";
        case LD2402_ERR_BAD_ARG:       return "bad_arg";
        case LD2402_ERR_NOT_CONNECTED: return "not_connected";
    }
    return "unknown";
}

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
// Counts every byte read during a config exchange, so a failed wait can tell
// "the module said nothing at all" (dead, unpowered, TX not landing on our RX)
// apart from "the module is talking but never sent the answer". Those look
// identical from a bool and want opposite reactions -- check the cable, versus
// try again. Only touched under s_uart_mutex.
static uint32_t s_exchangeBytes = 0;

static bool uartReadByte(uint8_t *b, TickType_t timeout) {
    if (uart_read_bytes(UART_PORT, b, 1, timeout) != 1) return false;
    s_exchangeBytes++;
    return true;
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
    const uint32_t bytesAtStart = s_exchangeBytes;
    // Silence and noise are different faults. If not one byte arrived while we
    // waited, the module is not talking to us at all.
    auto quiet = [&]() {
        return fail(s_exchangeBytes == bytesAtStart ? LD2402_ERR_NOT_CONNECTED
                                                    : LD2402_ERR_TIMEOUT);
    };
    while (esp_timer_get_time() < deadline) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = (uint16_t)((deadline - esp_timer_get_time()) / 1000);
        if (!readFrameBlocking(gotWord, body, bodyLen, sizeof(body), remaining)) return quiet();
        if (gotWord != wantWord) continue;   // stray frame, keep waiting
        if (bodyLen < 2) return fail(LD2402_ERR_BAD_REPLY);
        uint16_t status = body[0] | ((uint16_t)body[1] << 8);
        // The module answered the right command and said no. Nothing about
        // retrying will change that -- the value asked for was rejected.
        if (status != 0) return fail(LD2402_ERR_REFUSED);
        if (extra && extraLen) {
            uint16_t n = bodyLen - 2;
            if (n > extraCap) n = extraCap;
            memcpy(extra, body + 2, n);
            *extraLen = n;
        }
        return true;
    }
    return quiet();
}

static bool waitEvent(uint16_t word, uint16_t timeoutMs) {
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    static uint8_t body[200];
    const uint32_t bytesAtStart = s_exchangeBytes;
    auto quiet = [&]() {
        return fail(s_exchangeBytes == bytesAtStart ? LD2402_ERR_NOT_CONNECTED
                                                    : LD2402_ERR_TIMEOUT);
    };
    while (esp_timer_get_time() < deadline) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = (uint16_t)((deadline - esp_timer_get_time()) / 1000);
        if (!readFrameBlocking(gotWord, body, bodyLen, sizeof(body), remaining)) return quiet();
        if (gotWord == word) return true;
    }
    return quiet();
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

// The module's `state` byte from the engineering frame. It IS the enum the
// vendor manual (Table 5-7) documents:
//
//     0x00 nobody, 0x01 someone moving, 0x02 someone still
//
// Measured, not assumed. This driver spent a long time believing otherwise:
// the firmware reverse-engineering writeup (LD2402_firmware_RE.md §11c)
// decompiled the frame builder as
//
//     state = presence;                 // 0 or 1
//     if (...) state += 0x10;
//
// and concluded only 0x00/0x01/0x10/0x11 could appear, never 0x02 -- so
// moving/still had to be *derived* here by comparing per-gate energies
// against per-gate thresholds. An entire classification layer was built on
// that, plus a threshold cache to feed it and a degraded-mode fallback for
// when the cache was cold.
//
// Logging the raw byte on hardware (module fw v3.3.5, serial
// 26053356000016 -- the same version that writeup disassembled) showed
// 0x00, 0x01 AND 0x02 in normal use, and never 0x10 or 0x11. The writeup is
// wrong about the very build it analysed; it missed a code path. The manual
// was right all along.
//
// So the classifier is gone and this byte is read directly. The module's own
// still detector is a CIC/breathing filter over a long window (writeup §10) --
// strictly better than the energy-vs-threshold compare that replaced it.
//
// If this is ever in doubt again: log the byte. Do not re-derive it from a
// document, including this comment.
#define LD2402_STATE_NOBODY 0x00
#define LD2402_STATE_MOVING 0x01
#define LD2402_STATE_STILL  0x02
// How much payload one command may carry. The module reports this when
// config mode is entered (0x0020 = 32 bytes on v3.3.5); this default is what
// is assumed until it does. It sets how many parameters fit in one batched
// read or write -- see ld2402_read_parameters_raw().
#define CMD_BUFSIZE_DEFAULT 32
#define CMD_BUFSIZE_MAX     64
static uint16_t s_cmd_bufsize = CMD_BUFSIZE_DEFAULT;

static uint8_t s_state = 0;
static uint16_t s_distanceCm = 0;
static bool s_engineering = false;
static uint32_t s_energy[32] = {0};

// Per-gate thresholds, mirrored here so the write paths can skip a flash
// write when nothing changed, without a UART round trip to check. Kept in
// step by ld2402_cache_thresholds(), called after any read or write of
// them. Invalid until the first read lands, and the writer falls back
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

// Every distinct state byte seen since boot. Read by publishReading() to tell
// "this module does not report stillness" from "it is not still right now".
static volatile uint32_t s_state_seen = 0;

// State debounce -- see ld2402_set_state_debounce_ms() in the header.
static volatile uint16_t s_debounce_ms = 500;
static ld2402_activity_t s_published_activity = LD2402_ABSENT;
static ld2402_activity_t s_pending_activity = LD2402_ABSENT;
static int64_t s_pending_since_us = 0;

void ld2402_set_state_debounce_ms(uint16_t ms) { s_debounce_ms = ms; }
uint16_t ld2402_get_state_debounce_ms(void) { return s_debounce_ms; }

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
// The write paths below skip a flash write when the value asked for matches
// what the cache says is already there, so a stale cache means real threshold
// writes silently dropped as redundant.
//
// It used to matter twice over: publishReading() also classified moving vs
// still against it. That classifier is gone (the module reports the answer
// directly -- see the state-byte comment), so this is now purely a
// write-deduplication cache. It still has to be invalidated correctly.
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

// Two different questions, deliberately kept apart -- conflating them was a
// real bug.
//
//   s_quiet_log_until_us : "don't call this silence a dropout." Set whenever
//                          the stream is expected to be interrupted, which
//                          includes every ordinary config session.
//   s_busy_until_us      : "the module will refuse commands until then." Set
//                          only after calibration and auto-gain, which leave
//                          it genuinely unresponsive for a few seconds.
//
// Only the second is ever waited on. When exiting a plain config session also
// set the second one, a multi-step write -- engineering mode, then max range,
// then delay, then 32 thresholds, each its own session -- made every step wait
// out the previous step's window. That turned a restore into 15-20 seconds and
// pushed it past the app's timeout, so the app reported failure while the
// device carried on and applied everything.
static volatile int64_t s_quiet_log_until_us = 0;
static volatile int64_t s_busy_until_us = 0;

// What the module is busy with, in words a user would recognise. Null when
// nothing has claimed it.
static const char *s_quiet_reason = nullptr;

// True between a successful enable_config() and its end_config(). The module
// does not stream while config mode is held, which is expected silence.
static volatile bool s_in_config = false;

// How far through a bulk threshold write we are. Exposed because the write is
// a single driver call taking ten seconds or so, and a caller showing a
// progress bar cannot see inside it otherwise -- 0 then 100 is not progress.
static volatile int s_bulk_done = 0;
static volatile int s_bulk_total = 0;

// The stream will be interrupted; don't report it as a fault.
static void expect_stream_gap_for(int64_t seconds) {
    s_quiet_log_until_us = esp_timer_get_time() + seconds * 1000000LL;
}

// The module itself will refuse commands; anything wanting a config session
// must wait. Implies a stream gap too.
static void expect_module_busy_for(int64_t seconds) {
    s_busy_until_us = esp_timer_get_time() + seconds * 1000000LL;
    expect_stream_gap_for(seconds);
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

// Bumped each time the module returns from a silence nobody asked for -- see
// ld2402_connect_generation() in the header.
static volatile uint32_t s_connect_generation = 0;

uint32_t ld2402_connect_generation(void) { return s_connect_generation; }

bool ld2402_get_cached_disappear_delay_s(uint16_t *seconds) {
    if (s_disappear_delay_s < 0) return false;
    if (seconds) *seconds = (uint16_t)s_disappear_delay_s;
    return true;
}


// What engineering mode *should* be, as last requested. The watchdog restores
// it after the module reboots itself. Defaults to on: streaming the energy
// gates is the reason to use this driver over the module's plain IO pin.
static volatile bool s_want_engineering = true;

// Set while another output mode is being observed, so the watchdog does not
// undo the experiment five seconds in.
static volatile bool s_watchdog_suspended = false;
static uint32_t s_byteCount = 0;
static int64_t s_lastByteUs = 0;
static int64_t s_lastUpdateUs = 0;

static void publishReading() {
    ld2402_reading_t r;
    r.presence = (s_state != LD2402_STATE_NOBODY);
    r.connected = true;   // ld2402_get_reading() recomputes this from staleness
    r.distance_cm = s_distanceCm;
    r.engineering = s_engineering;
    for (uint8_t i = 0; i < 16; i++) {
        r.trigger_db[i] = s_engineering ? dbFromRaw(s_energy[i]) : NAN;
        r.motionless_db[i] = s_engineering ? dbFromRaw(s_energy[16 + i]) : NAN;
    }

    // Moving vs still comes straight from the module. See the state-byte
    // comment above for why this replaced a derived classifier.
    //
    // In ASCII mode there is no state byte at all -- handleTextLine() can only
    // set NOBODY or MOVING -- so a still person reads as moving there. That is
    // a real limit of text mode, not a fallback worth writing code for: the
    // driver keeps the module in engineering mode precisely so this does not
    // happen.
    r.moving = (s_state == LD2402_STATE_MOVING);
    r.still  = (s_state == LD2402_STATE_STILL);

    // Fall back to deriving stillness on modules that never report it.
    //
    // This driver used to derive moving/still from the gate energies, and that
    // was replaced by reading the state byte after a module was observed
    // sending 0x02 -- a better answer where it exists, since the module's own
    // still detector is a long-window breathing filter that no threshold
    // comparison here can match.
    //
    // The mistake was generalising from that one module. Measured across four
    // parts on the same firmware v3.3.5: one emits 0x02, three never do in
    // ordinary use. On those three, reading the byte alone means stillness
    // simply does not exist -- a person sitting quietly reads as moving, or as
    // nobody once the disappear delay expires.
    //
    // So: prefer the module's answer, and derive only for modules that have
    // never given one. s_state_seen makes that distinction cheap and honest --
    // it is a record of every state byte since boot, so "has never reported
    // still" is a fact rather than an assumption about the current frame.
    //
    // The derivation is the original one: present, and no movement gate over
    // its threshold. Gate 0 is skipped because near-field clutter there would
    // otherwise pin `moving` true forever -- which is exactly what a board
    // with its own PCB in the antenna's near field does.
    // Presence is left exactly as the module reported it -- only the
    // moving/still label is derived.
    //
    // A version of this derived presence too, from the gate energies, and it
    // was worse in the way that matters: the energies of someone sitting
    // quietly hover around their threshold, so presence itself dropped out
    // frame to frame and the room read as empty with the person still in it.
    // Losing presence turns a light off; mislabelling moving as still only
    // changes a word. The module's bit has its disappear delay behind it and
    // is far steadier, so it decides whether anyone is there, and this decides
    // only what they are doing.
    if (!r.still && !(s_state_seen & (1u << LD2402_STATE_STILL))) {
        // Presence is whatever the module said; only the label is derived.
        r.moving = false;
        if (!r.presence) {
            // Nothing to classify. The gate scan used to run regardless, so a
            // stray gate over threshold with nobody detected produced
            // moving=true, presence=false -- a combination that cannot be true
            // and that no consumer could render.
        } else if (s_engineering && s_thresholds_valid) {
            // Mirrors the module's own peak selection: scan gates 1..15 and
            // take any gate whose energy clears that gate's threshold. Gate 0
            // is skipped for the same reason the module skips it -- its
            // threshold is never evaluated, so including it would let
            // near-field clutter set a flag no configuration could clear.
            for (uint8_t g = 1; g < 16; g++) {
                if (!isnan(r.trigger_db[g]) && r.trigger_db[g] > s_trigger_th[g]) {
                    r.moving = true;
                    break;
                }
            }
        } else {
            // No engineering frame or no threshold cache yet: presence is all
            // there is, so report it as movement rather than inventing a
            // classification. Guessing "still" here is what once made a
            // walking person read as stationary.
            r.moving = true;
        }
        // Still is the complement of moving, not its own threshold test.
        //
        // Testing the motionless energies separately looks more principled and
        // isn't: the two chains are the same signal under different filters,
        // so both can clear, neither can, or either alone can -- four
        // combinations for a two-way question, and two of them have nothing
        // sensible to display. Something is there, and it is either moving or
        // it isn't.
        r.still = r.presence && !r.moving;
    }
    // One value carrying the same decision, so callers that want a single
    // answer do not have to recombine the booleans and risk inventing a
    // fourth state that cannot occur.
    // Recomputed after the derivation above, which can change all three.
    r.activity = !r.presence ? LD2402_ABSENT : (r.moving ? LD2402_MOVING : LD2402_STILL);

    // Debounce -- see ld2402_set_state_debounce_ms().
    //
    // The booleans are brought back into line with the published activity
    // afterwards rather than debounced separately: three flags settling at
    // different moments is how a reading ends up claiming to be moving and
    // absent at once.
    if (s_debounce_ms) {
        const int64_t now = esp_timer_get_time();
        // Only moving<->still is debounced, in both directions.
        //
        // That is the pair that flickers: the two chains are the same signal
        // under different filters, so someone shifting in a chair crosses back
        // and forth several times a second.
        //
        // Anything involving absent is passed straight through. Presence comes
        // from the module, which already holds it for the disappear delay --
        // the setting that exists for exactly this -- so holding it again here
        // would silently extend a delay the user had already chosen, and make
        // arrivals late for no reason.
        // Still is the sticky state: leaving it has to be sustained.
        //
        // Someone sitting quietly is the case that fluctuates. Their gate
        // energies sit near the thresholds and cross them for a frame at a
        // time, so the raw classification jumps out to moving, or drops to
        // absent, and straight back -- tens of times a minute on a person who
        // has not actually done anything. Holding the exit is what makes the
        // published state describe the room rather than the noise.
        //
        // Everything else is immediate: arriving, and settling from moving
        // into still. Only departures from a published still wait, and only
        // for as long as the fluctuation lasts -- a real change persists and
        // is published as soon as the debounce elapses.
        // Both departures from a published still are held, for different
        // lengths, because they are different kinds of event.
        //
        // Dropping to absent is almost always the signal dipping under
        // threshold for a frame on someone sitting right there, and publishing
        // it makes the room read as empty with a person in it -- so it gets
        // the full debounce.
        //
        // Movement is usually real and is the edge everything downstream
        // reacts to, so a long hold is felt immediately as lag. But a single
        // frame over threshold is not movement either. A quarter of the
        // debounce, capped at half a second, rejects the one-frame blips
        // without being perceptible: at the 2 s default that is 500 ms.
        // One hold, and only between moving and still.
        //
        // Absent is not debounced here at all: the module's own disappear
        // delay already decides how long the room stays occupied after motion
        // stops, and that is the setting for the job. A second hold on top
        // duplicated it, silently added to it, and needed the two summed
        // together to explain what the device would do -- a total you have to
        // explain is a sign there is one setting too many.
        //
        // What is left is the part the module cannot do: it reports moving and
        // still from the same signal under two filters, so a person shifting
        // in a chair crosses between them for a frame at a time. This holds
        // that, in both directions -- holding one way only just moves the
        // flicker to the other, which showed up as the light going moving,
        // still, moving on a single walk-in.
        const bool between_active =
            (r.activity == LD2402_MOVING || r.activity == LD2402_STILL) &&
            (s_published_activity == LD2402_MOVING || s_published_activity == LD2402_STILL);
        const uint32_t hold_ms = s_debounce_ms;
        const bool arriving = !between_active;

        if (arriving) {
            // An arrival is movement, whatever the first frame classified it
            // as. You have to move to arrive: a presence event that begins
            // with someone already motionless is not a thing that happens.
            //
            // It happens here because the first frame is published unheld --
            // presence must not wait -- so a single frame that fails to clear
            // a movement threshold makes the whole event start as still. Near
            // the sensor that is not even unlikely: the gates covering the
            // first metre or so can carry thresholds no person clears.
            //
            // Stated rather than held, so presence is still reported at once.
            // Stillness follows a moment later if they really have settled.
            s_published_activity = LD2402_MOVING;
            s_pending_activity = LD2402_MOVING;
            s_pending_since_us = now;
        } else if (r.activity != s_published_activity) {
            if (r.activity != s_pending_activity) {
                s_pending_activity = r.activity;
                s_pending_since_us = now;
            } else if (now - s_pending_since_us >= (int64_t)hold_ms * 1000) {
                s_published_activity = r.activity;
            }
        } else {
            // Back to what is already published: cancel any pending change.
            s_pending_activity = r.activity;
            s_pending_since_us = now;
        }
        r.activity = s_published_activity;
        r.moving   = (r.activity == LD2402_MOVING);
        r.still    = (r.activity == LD2402_STILL);
        r.presence = (r.activity != LD2402_ABSENT);
    }
    r.bytes_received = s_byteCount;
    r.last_byte_us = s_lastByteUs;
    r.last_update_us = s_lastUpdateUs;
    setReading(r);
}

static void handleTextLine(void) {
    // Trim trailing \r/space already excluded by feedByte(); just null-terminate.
    if (s_lineLen == 0) return;
    if (strcmp(s_lineBuf, "OFF") == 0) {
        s_state = LD2402_STATE_NOBODY;
        s_distanceCm = 0;
        s_engineering = false;
        s_lastUpdateUs = esp_timer_get_time();
        publishReading();
        return;
    }
    const char *colon = strchr(s_lineBuf, ':');
    if (strncmp(s_lineBuf, "distance", 8) == 0 && colon) {
        // Text mode reports presence and distance only -- there is no state
        // byte, so a still person is indistinguishable from a moving one here.
        s_state = LD2402_STATE_MOVING;
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

uint8_t ld2402_debug_raw_state(void) { return s_state; }

uint32_t ld2402_debug_state_seen_mask(void) { return s_state_seen; }

// The last engineering frame body, kept verbatim. 131 is the documented size
// (1 state + 2 distance + 128 energies); the extra few bytes are so a frame
// that is not that size is still captured rather than silently clipped to
// look like one that is.
static uint8_t s_last_frame[144];
static uint16_t s_last_frame_len = 0;
static uint16_t s_last_frame_full = 0;

size_t ld2402_debug_last_frame(uint8_t *out, size_t max, uint16_t *frame_len) {
    if (frame_len) *frame_len = s_last_frame_full;
    if (!out || !s_last_frame_len) return 0;
    const size_t n = s_last_frame_len < max ? s_last_frame_len : max;
    memcpy(out, s_last_frame, n);
    return n;
}

static void handleEngineeringFrame(const uint8_t *body, uint16_t len) {
    if (len < 3) return;
    // The whole frame at debug level, for when the decoded reading and the
    // hardware disagree. Off unless this component's log level is raised, so
    // it costs a level check per frame the rest of the time.
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, body, len, ESP_LOG_DEBUG);
    s_state = body[0];
    s_state_seen |= 1u << (s_state < 31 ? s_state : 31);
    // Keep the bytes as well as the decode, for reading against the manual's
    // frame diagram when the two appear to disagree.
    s_last_frame_len = len < sizeof(s_last_frame) ? len : sizeof(s_last_frame);
    memcpy(s_last_frame, body, s_last_frame_len);
    s_last_frame_full = len;
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
// The last bytes off the UART, before any parsing.
//
// handleEngineeringFrame() only ever sees the body: the state machine below
// validates and consumes the F4 F3 F2 F1 header, the length and the
// F8 F7 F6 F5 footer, so a dump taken there cannot show the framing the manual
// documents -- nor anything the parser rejected, which is exactly what you
// want when frames are being dropped. This sits ahead of all of it.
//
// 320 bytes is a little over two whole 141-byte packets, so a dump always
// contains at least one complete frame with its framing intact.
static uint8_t s_raw[320];
static uint16_t s_raw_head = 0;
static bool s_raw_wrapped = false;

size_t ld2402_debug_raw_stream(uint8_t *out, size_t max) {
    if (!out) return 0;
    // Oldest first, so the result reads in the order the bytes arrived.
    const size_t have = s_raw_wrapped ? sizeof(s_raw) : s_raw_head;
    const size_t n = have < max ? have : max;
    for (size_t i = 0; i < n; i++) {
        const size_t idx = (s_raw_head + sizeof(s_raw) - n + i) % sizeof(s_raw);
        out[i] = s_raw[idx];
    }
    return n;
}

static void feedByte(uint8_t b) {
    s_byteCount++;
    s_lastByteUs = esp_timer_get_time();
    s_raw[s_raw_head] = b;
    if (++s_raw_head >= sizeof(s_raw)) { s_raw_head = 0; s_raw_wrapped = true; }
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
// arrays stop, the state byte disappears so a still person reads as moving,
// and the tuning screen shows its "engineering data is off" banner with no
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
    // Whether the current silence is one we asked for, so the return can be
    // reported as a resume rather than a recovery.
    bool paused_deliberately = false;
    int64_t down_since_us = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        ld2402_reading_t r;
        ld2402_get_reading(&r);

        // ── module presence on the UART ──
        //
        // Calibration and auto-gain both stop the module streaming for longer
        // than the 2s freshness window, so without help this reported "sensor
        // module stopped responding ... back after 5s silent" after every
        // routine tune -- describing a fault that had not happened, in the
        // same words as the real dropouts this exists to catch.
        //
        // Saying nothing at all would be worse in a different way: the sensor
        // genuinely does stop reporting for a few seconds, and a log that
        // skips it leaves an unexplained gap. So it says which it is.
        const bool expected_quiet = s_calibrating || s_autogain_running || s_in_config ||
                                    esp_timer_get_time() < s_quiet_log_until_us;

        if (r.connected != was_connected) {
            if (!r.connected) {
                down_since_us = esp_timer_get_time();
                if (expected_quiet && s_quiet_reason) {
                    notify("sensor paused for %s", s_quiet_reason);
                    paused_deliberately = true;
                } else {
                    notify("sensor module stopped responding");
                    paused_deliberately = false;
                    // Forget everything cached about it.
                    //
                    // Silence nobody asked for has two causes and they are
                    // indistinguishable from here: the module rebooted, or it
                    // was unplugged and a different one put in its place. The
                    // second is now easy to do and makes every cached value a
                    // statement about the wrong part -- including the ones
                    // used to skip redundant flash writes, so a Save against
                    // the new module gets silently dropped as "already that
                    // value". Re-reading costs one config session; being
                    // wrong costs a setting that never applied.
                    s_max_distance_m = -1;
                    s_disappear_delay_s = -1;
                    invalidate_threshold_cache();
                }
            } else {
                if (!paused_deliberately) s_connect_generation++;
                if (paused_deliberately) {
                    notify("sensor resumed after %s",
                           s_quiet_reason ? s_quiet_reason : "pause");
                    paused_deliberately = false;
                } else if (first_pass) {
                    notify("sensor module connected");
                } else {
                    notify("sensor module back after %llds silent",
                           (esp_timer_get_time() - down_since_us) / 1000000);
                }
            }
            was_connected = r.connected;
            first_pass = false;
        }

        if (!s_want_engineering || s_watchdog_suspended) continue;

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
// every later threshold write unable to dedupe, from the first calibration
// until the next reboot. So it stays resident and re-primes instead, which
// costs one sleeping task.
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
                // Detection itself is unaffected -- the module classifies
                // moving/still on its own. Without this cache the driver just
                // can't skip redundant threshold writes, and the tuning screen
                // has to do a slow read to show current values.
                ESP_LOGW(TAG, "threshold cache unavailable -- writes won't dedupe");
                notify("could not read gate thresholds -- settings writes will be slower");
            }
            continue;
        }
        ld2402_read_all_thresholds(nullptr, nullptr, 1000);
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
    // Fill the threshold cache from boot rather than only once someone opens
    // the tuning screen, so the first settings write can already dedupe. Its
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
    if (s_calibrating || s_autogain_running) return fail(LD2402_ERR_BUSY);

    const int64_t wait_until =
        (s_busy_until_us < esp_timer_get_time() + 6000000LL) ? s_busy_until_us
                                                             : esp_timer_get_time() + 6000000LL;
    while (esp_timer_get_time() < wait_until && !s_calibrating && !s_autogain_running) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Session lock taken here, released by ld2402_end_config() -- see the
    // s_session_mutex comment above. If the handshake below never succeeds,
    // release it before returning; nothing will call end_config to do it for
    // us, and leaking it would starve every future config call forever.
    if (xSemaphoreTake(s_session_mutex, pdMS_TO_TICKS((uint32_t)timeoutMs + 1000)) != pdTRUE)
        return fail(LD2402_ERR_BUSY);

    UartSession s;
    if (!s.held) { xSemaphoreGive(s_session_mutex); return fail(LD2402_ERR_BUSY); }

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
        uint8_t hello[8];
        uint16_t helloLen = 0;
        if (waitAck(0x00FF, 250, hello, sizeof(hello), &helloLen)) {
            // The ACK carries the protocol version and the module's command
            // buffer size (manual 5.2.2). The buffer size is how much payload
            // one command may carry, which is what makes batching safe: read
            // and write take N records per frame, and this is the only thing
            // that says how many N may be.
            if (helloLen >= 4) {
                uint16_t bufsize = hello[2] | ((uint16_t)hello[3] << 8);
                // Believe it only within reason. A garbled ACK reporting a
                // huge buffer would build frames the module then rejects, and
                // one reporting 0 would divide the batch size to nothing.
                if (bufsize >= 8 && bufsize <= CMD_BUFSIZE_MAX) s_cmd_bufsize = bufsize;
            }
            // Config mode stops the module streaming, for as long as the
            // session is held -- a bulk threshold write is ten seconds of it.
            // Without a reason set, that reads as "stopped responding", which
            // is the wording reserved for silence nobody asked for.
            s_quiet_reason = "a settings write";
            s_in_config = true;
            return true;   // session_mutex stays held until end_config()
        }
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
    s_in_config = false;
    // A logging grace only. Deliberately NOT expect_module_busy_for(): the
    // module accepts commands again immediately after a plain config exit, and
    // making the next session wait here is what made multi-step writes crawl.
    expect_stream_gap_for(4);
    xSemaphoreGive(s_session_mutex);
    return ok;
}

bool ld2402_read_firmware_version(char *buf, size_t cap, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
    if (cap == 0) return fail(LD2402_ERR_BAD_ARG);
    sendCommand(0x0000, nullptr, 0);
    uint8_t extra[64];
    uint16_t n = 0;
    if (!waitAck(0x0000, timeoutMs, extra, sizeof(extra), &n)) return false;
    if (n < 2) return fail(LD2402_ERR_BAD_REPLY);
    uint16_t verLen = extra[0] | ((uint16_t)extra[1] << 8);
    if (verLen > n - 2) verLen = n - 2;
    if (verLen > cap - 1) verLen = cap - 1;
    memcpy(buf, extra + 2, verLen);
    buf[verLen] = '\0';
    return true;
}

bool ld2402_read_serial_number(char *buf, size_t cap, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
    if (cap == 0) return fail(LD2402_ERR_BAD_ARG);
    sendCommand(0x0011, nullptr, 0);
    uint8_t extra[40];
    uint16_t n = 0;
    if (!waitAck(0x0011, timeoutMs, extra, sizeof(extra), &n)) return false;
    if (n < 2) return fail(LD2402_ERR_BAD_REPLY);
    uint16_t snLen = extra[0] | ((uint16_t)extra[1] << 8);
    if (snLen > n - 2) snLen = n - 2;
    if (snLen > cap - 1) snLen = cap - 1;
    memcpy(buf, extra + 2, snLen);
    buf[snLen] = '\0';
    return true;
}

void ld2402_suspend_engineering_watchdog(bool suspend) {
    s_watchdog_suspended = suspend;
}

size_t ld2402_debug_command(uint16_t word, const uint8_t *payload, size_t payloadLen,
                            uint8_t *reply, size_t replyMax, uint16_t timeoutMs) {
    if (!ld2402_enable_config(timeoutMs)) return 0;
    size_t got = 0;
    {
        UartSession s;
        if (s.held) {
            sendCommand(word, payload, (uint16_t)payloadLen);
            // waitAck hands back the payload after the 2-byte echoed command
            // word -- which is where a status field lives, and what the
            // manual's byte offsets are counted against.
            uint16_t n = 0;
            static uint8_t extra[200];
            if (waitAck(word, timeoutMs, extra, sizeof(extra), &n)) {
                got = n < replyMax ? n : replyMax;
                memcpy(reply, extra, got);
            }
        }
    }
    ld2402_end_config(timeoutMs);
    return got;
}

bool ld2402_set_output_mode_raw(uint32_t value, uint16_t timeoutMs) {
    if (!ld2402_enable_config(timeoutMs)) return false;
    bool ok;
    {
        UartSession s;
        if (!s.held) { ld2402_end_config(timeoutMs); return fail(LD2402_ERR_BUSY); }
        uint8_t val[6] = {0x00, 0x00,
                          (uint8_t)value, (uint8_t)(value >> 8),
                          (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
        sendCommand(0x0012, val, 6);
        ok = waitAck(0x0012, 1000);
    }
    ld2402_end_config(timeoutMs);
    // The parser decides what it is looking at from the bytes themselves, but
    // s_engineering is only cleared when a text line arrives -- so a mode that
    // emits neither would otherwise leave it reading true forever.
    if (ok && value != 4) s_engineering = false;
    ESP_LOGW(TAG, "output mode set to %u (experiment)", (unsigned)value);
    return ok;
}

bool ld2402_set_output_mode(bool engineering, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
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
    // Bail if config mode was refused. Ignoring this meant the commands
    // below were sent to a module that was not listening: every one of
    // them failed, and a restore of all 32 thresholds reported "sensor
    // did not ack" having written nothing. It also called end_config(),
    // releasing a session lock this call never took.
    if (!ld2402_enable_config(configTimeoutMs)) return false;
    bool ok = ld2402_set_output_mode(on, 1000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_read_parameter_raw(uint16_t id, uint32_t *value, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
    uint8_t val[2] = {(uint8_t)(id & 0xFF), (uint8_t)(id >> 8)};
    sendCommand(0x0008, val, 2);
    uint8_t extra[4];
    uint16_t n = 0;
    if (!waitAck(0x0008, timeoutMs, extra, sizeof(extra), &n)) return false;
    if (n < 4) return fail(LD2402_ERR_BAD_REPLY);
    *value = (uint32_t)extra[0] | ((uint32_t)extra[1] << 8) | ((uint32_t)extra[2] << 16) | ((uint32_t)extra[3] << 24);
    return true;
}

// Batched read. The manual's read command (0x0008) takes N parameter ids and
// answers with N values -- "(2-byte parameter ID) * N" in, "(4-byte parameter
// value) * N" out (manual 5.2.6). This driver sent exactly one per command
// for a long time, which made fetching all 32 thresholds 32 separate
// command/ACK round trips and is most of why that took seconds.
//
// Split into as many frames as the module's buffer allows, transparently, so
// callers just ask for what they want.
bool ld2402_read_parameters_raw(const uint16_t *ids, uint32_t *values,
                                 uint8_t count, uint16_t timeoutMs) {
    if (!ids || !values || count == 0) return fail(LD2402_ERR_BAD_ARG);
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);

    const uint8_t perFrame = (uint8_t)(s_cmd_bufsize / 2);   // 2 bytes per id
    uint8_t done = 0;
    while (done < count) {
        uint8_t n = count - done;
        if (n > perFrame) n = perFrame;

        uint8_t req[CMD_BUFSIZE_MAX];
        for (uint8_t i = 0; i < n; i++) {
            req[i * 2]     = (uint8_t)(ids[done + i] & 0xFF);
            req[i * 2 + 1] = (uint8_t)(ids[done + i] >> 8);
        }
        sendCommand(0x0008, req, (uint16_t)(n * 2));

        uint8_t extra[CMD_BUFSIZE_MAX * 2];
        uint16_t got = 0;
        if (!waitAck(0x0008, timeoutMs, extra, sizeof(extra), &got)) return false;
        // Short reply means the module answered fewer than asked. Treating a
        // partial answer as success would hand back stale stack contents for
        // the rest, which is worse than failing.
        if (got < (uint16_t)(n * 4)) return fail(LD2402_ERR_BAD_REPLY);

        for (uint8_t i = 0; i < n; i++) {
            const uint8_t *p = extra + i * 4;
            values[done + i] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                                ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        }
        done += n;
    }
    return true;
}

// Batched write, same idea: "(2-byte parameter ID + 4-byte parameter value)
// * N" (manual 5.2.7). Six bytes per record, so fewer fit per frame than on
// the read side.
bool ld2402_set_parameters_raw(const uint16_t *ids, const uint32_t *values,
                                uint8_t count, uint16_t timeoutMs) {
    if (!ids || !values || count == 0) return fail(LD2402_ERR_BAD_ARG);
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);

    const uint8_t perFrame = (uint8_t)(s_cmd_bufsize / 6);   // 6 bytes per record
    uint8_t done = 0;
    while (done < count) {
        uint8_t n = count - done;
        if (n > perFrame) n = perFrame;

        uint8_t req[CMD_BUFSIZE_MAX];
        for (uint8_t i = 0; i < n; i++) {
            uint8_t *p = req + i * 6;
            const uint16_t id = ids[done + i];
            const uint32_t v  = values[done + i];
            p[0] = (uint8_t)(id & 0xFF);
            p[1] = (uint8_t)(id >> 8);
            p[2] = (uint8_t)v;
            p[3] = (uint8_t)(v >> 8);
            p[4] = (uint8_t)(v >> 16);
            p[5] = (uint8_t)(v >> 24);
        }
        sendCommand(0x0007, req, (uint16_t)(n * 6));
        if (!waitAck(0x0007, timeoutMs)) return false;
        done += n;
    }
    return true;
}

bool ld2402_set_parameter_raw(uint16_t id, uint32_t value, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
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
    // Bail if config mode was refused. Ignoring this meant the commands
    // below were sent to a module that was not listening: every one of
    // them failed, and a restore of all 32 thresholds reported "sensor
    // did not ack" having written nothing. It also called end_config(),
    // releasing a session lock this call never took.
    if (!ld2402_enable_config(configTimeoutMs)) return false;
    bool ok = ld2402_set_max_distance_m(meters, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_set_and_save_disappear_delay_s(uint16_t seconds, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (s_disappear_delay_s >= 0 && s_disappear_delay_s == (int32_t)seconds) return true;
    // Bail if config mode was refused. Ignoring this meant the commands
    // below were sent to a module that was not listening: every one of
    // them failed, and a restore of all 32 thresholds reported "sensor
    // did not ack" having written nothing. It also called end_config(),
    // releasing a session lock this call never took.
    if (!ld2402_enable_config(configTimeoutMs)) return false;
    bool ok = ld2402_set_disappear_delay_s(seconds, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_set_trigger_threshold_db(uint8_t gate, float db, uint16_t timeoutMs) {
    if (gate > 15) return fail(LD2402_ERR_BAD_ARG);
    if (!ld2402_set_parameter_raw(0x0010 + gate, rawFromDb(db), timeoutMs)) return false;
    // Mirror into the classifier's cache. Doing it in the primitives means
    // every path -- single gate write, bulk save, calibration read-back --
    // keeps the cache honest without each caller remembering to.
    s_trigger_th[gate] = db;
    return true;
}
bool ld2402_read_trigger_threshold_db(uint8_t gate, float *db, uint16_t timeoutMs) {
    if (gate > 15) return fail(LD2402_ERR_BAD_ARG);
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0010 + gate, &raw, timeoutMs)) return false;
    *db = dbFromRaw(raw);
    s_trigger_th[gate] = *db;
    s_trigger_seen |= (uint16_t)1u << gate;
    note_thresholds_progress();
    return true;
}
bool ld2402_set_motionless_threshold_db(uint8_t gate, float db, uint16_t timeoutMs) {
    if (gate > 15) return fail(LD2402_ERR_BAD_ARG);
    if (!ld2402_set_parameter_raw(0x0030 + gate, rawFromDb(db), timeoutMs)) return false;
    s_motionless_th[gate] = db;
    return true;
}
bool ld2402_read_motionless_threshold_db(uint8_t gate, float *db, uint16_t timeoutMs) {
    if (gate > 15) return fail(LD2402_ERR_BAD_ARG);
    uint32_t raw;
    if (!ld2402_read_parameter_raw(0x0030 + gate, &raw, timeoutMs)) return false;
    *db = dbFromRaw(raw);
    s_motionless_th[gate] = *db;
    s_motionless_seen |= (uint16_t)1u << gate;
    note_thresholds_progress();
    return true;
}

bool ld2402_set_and_save_trigger_threshold_db(uint8_t gate, float db, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (gate > 15) return fail(LD2402_ERR_BAD_ARG);
    if (s_thresholds_valid && fabsf(s_trigger_th[gate] - db) < 0.05f) return true;
    // Bail if config mode was refused. Ignoring this meant the commands
    // below were sent to a module that was not listening: every one of
    // them failed, and a restore of all 32 thresholds reported "sensor
    // did not ack" having written nothing. It also called end_config(),
    // releasing a session lock this call never took.
    if (!ld2402_enable_config(configTimeoutMs)) return false;
    bool ok = ld2402_set_trigger_threshold_db(gate, db, 1000);
    if (ok) ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
    ld2402_end_config(configTimeoutMs);
    return ok;
}

bool ld2402_set_and_save_motionless_threshold_db(uint8_t gate, float db, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (gate > 15) return fail(LD2402_ERR_BAD_ARG);
    if (s_thresholds_valid && fabsf(s_motionless_th[gate] - db) < 0.05f) return true;
    // Bail if config mode was refused. Ignoring this meant the commands
    // below were sent to a module that was not listening: every one of
    // them failed, and a restore of all 32 thresholds reported "sensor
    // did not ack" having written nothing. It also called end_config(),
    // releasing a session lock this call never took.
    if (!ld2402_enable_config(configTimeoutMs)) return false;
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
// Uses the driver's threshold cache, so this costs a float comparison per
// gate rather than a UART read. The cache is only trusted once every
// gate has been read back at least once; before that, write everything,
// because "unknown" must never be mistaken for "unchanged".
void ld2402_bulk_write_progress(int *done, int *total) {
    if (done) *done = s_bulk_done;
    if (total) *total = s_bulk_total;
}

// All 32 thresholds in one go, using batched reads: two frames instead of 32
// command/ACK round trips. Caller must already hold a config session.
//
// Both outputs are optional -- passing null for each still primes the
// driver's cache, which is the only reason the priming task calls this.
bool ld2402_read_all_thresholds(float triggerDb[16], float motionlessDb[16],
                                 uint16_t timeoutMs) {
    uint16_t ids[16];
    uint32_t raw[16];

    for (uint8_t g = 0; g < 16; g++) ids[g] = (uint16_t)(0x0010 + g);
    if (!ld2402_read_parameters_raw(ids, raw, 16, timeoutMs)) return false;
    for (uint8_t g = 0; g < 16; g++) {
        const float db = dbFromRaw(raw[g]);
        if (triggerDb) triggerDb[g] = db;
        s_trigger_th[g] = db;
        s_trigger_seen |= (uint16_t)(1u << g);
    }

    for (uint8_t g = 0; g < 16; g++) ids[g] = (uint16_t)(0x0030 + g);
    if (!ld2402_read_parameters_raw(ids, raw, 16, timeoutMs)) return false;
    for (uint8_t g = 0; g < 16; g++) {
        const float db = dbFromRaw(raw[g]);
        if (motionlessDb) motionlessDb[g] = db;
        s_motionless_th[g] = db;
        s_motionless_seen |= (uint16_t)(1u << g);
    }

    note_thresholds_progress();
    return true;
}

bool ld2402_save_all_thresholds(const float triggerDb[16], const float motionlessDb[16],
                                uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    return ld2402_write_all_thresholds(triggerDb, motionlessDb, true,
                                        configTimeoutMs, saveTimeoutMs);
}

bool ld2402_write_all_thresholds(const float triggerDb[16], const float motionlessDb[16],
                                  bool commit, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    // Counted before anything is skipped, so the total matches what the caller
    // asked for rather than what turned out to need writing -- a bar that
    // shrinks its own scale mid-run is worse than no bar.
    s_bulk_total = (triggerDb ? 16 : 0) + (motionlessDb ? 16 : 0);
    s_bulk_done = 0;
    // Bail if config mode was refused. Ignoring this meant the commands
    // below were sent to a module that was not listening: every one of
    // them failed, and a restore of all 32 thresholds reported "sensor
    // did not ack" having written nothing. It also called end_config(),
    // releasing a session lock this call never took.
    if (!ld2402_enable_config(configTimeoutMs)) return false;
    bool ok = true;
    int written = 0;

    // 0.05dB: the module stores linear power and round-trips through a log,
    // so a value read back is never bit-identical to the one written. Any
    // real edit is far larger than this.
    auto same = [](float a, float b) { return fabsf(a - b) < 0.05f; };

    // Collect what actually changed, then write it in batched frames rather
    // than one command per gate. Unchanged gates are still counted towards
    // progress, so the bar reflects what the caller asked for.
    uint16_t ids[32];
    uint32_t vals[32];
    uint8_t n = 0;

    if (triggerDb) {
        for (uint8_t i = 0; i < 16; i++) {
            s_bulk_done++;
            if (s_thresholds_valid && same(triggerDb[i], s_trigger_th[i])) continue;
            ids[n] = (uint16_t)(0x0010 + i);
            vals[n] = rawFromDb(triggerDb[i]);
            n++;
        }
    }
    if (motionlessDb) {
        for (uint8_t i = 0; i < 16; i++) {
            s_bulk_done++;
            if (s_thresholds_valid && same(motionlessDb[i], s_motionless_th[i])) continue;
            ids[n] = (uint16_t)(0x0030 + i);
            vals[n] = rawFromDb(motionlessDb[i]);
            n++;
        }
    }

    if (n > 0) {
        ok = ld2402_set_parameters_raw(ids, vals, n, 1000);
        written = n;
        // Mirror into the cache only on success -- a cache claiming values the
        // module never accepted would make the retry skip them as unchanged,
        // which is exactly how a failed commit once left flash stale with
        // nothing reporting it.
        if (ok) {
            for (uint8_t i = 0; i < n; i++) {
                const uint16_t id = ids[i];
                const float db = dbFromRaw(vals[i]);
                if (id >= 0x0010 && id <= 0x001F) s_trigger_th[id - 0x0010] = db;
                else if (id >= 0x0030 && id <= 0x003F) s_motionless_th[id - 0x0030] = db;
            }
        }
    }

    if (written == 0) {
        ESP_LOGI(TAG, "thresholds unchanged -- no write");
    } else if (ok && !commit) {
        // Left in the module's RAM on purpose: in effect now, gone on its
        // next power cycle. For a control being dragged, that is the point --
        // no flash erase per adjustment.
        ESP_LOGI(TAG, "wrote %d of 32 thresholds (not committed)", written);
    } else if (ok) {
        ESP_LOGI(TAG, "wrote %d of 32 thresholds", written);
        ok = ld2402_save_parameters(saveTimeoutMs ? saveTimeoutMs : 2000);
        if (!ok) {
            // The writes applied but the commit did not. The cache mirrors
            // what was *sent*, so leaving it intact would make the retry look
            // like a no-op ("unchanged -- no write") and flash would stay
            // stale for good, with nothing on screen saying so. Dropping it
            // forces the next attempt to re-read the module and write again.
            ESP_LOGW(TAG, "commit refused -- dropping cache so a retry rewrites");
            invalidate_threshold_cache();
        }
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
    if (!s.held) return fail(LD2402_ERR_BUSY);
    uint16_t trig = (uint16_t)triggerFactor * 10, hold = (uint16_t)holdFactor * 10, micro = (uint16_t)microFactor * 10;
    uint8_t val[6] = {
        (uint8_t)trig, (uint8_t)(trig >> 8),
        (uint8_t)hold, (uint8_t)(hold >> 8),
        (uint8_t)micro, (uint8_t)(micro >> 8)};
    sendCommand(0x0009, val, 6);
    if (!waitAck(0x0009, timeoutMs)) return false;
    // Calibration rewrites all 32 thresholds inside the module, so every
    // cached copy here is now wrong. Nothing used to drop them, which left
    // the no-op guards on the write paths comparing against values the module
    // no longer holds -- silently skipping real writes.
    invalidate_threshold_cache();
    s_quiet_reason = "calibration";
    s_calibrating = true;
    return true;
}

bool ld2402_calibration_progress(uint8_t *percent, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
    sendCommand(0x000A, nullptr, 0);
    uint8_t extra[2];
    uint16_t n = 0;
    if (!waitAck(0x000A, timeoutMs, extra, sizeof(extra), &n)) return false;
    if (n < 2) return fail(LD2402_ERR_BAD_REPLY);
    *percent = extra[0];   // fits in one byte (0-100); extra[1] is always 0
    // Releases threshold_cache_task to go and re-read the thresholds the
    // calibration just rewrote. Polling this to completion is what every
    // caller already does, so there is no separate "calibration finished"
    // event to hang this off.
    if (*percent >= 100) {
        s_calibrating = false;
        expect_module_busy_for(10);
    }
    return true;
}

bool ld2402_read_calibration_interference(bool *hadInterference, uint16_t *gateMask, uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
    sendCommand(0x0014, nullptr, 0);
    const uint16_t wantWord = 0x0014 + 0x0100;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    static uint8_t body[200];
    while (esp_timer_get_time() < deadline) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = (uint16_t)((deadline - esp_timer_get_time()) / 1000);
        if (!readFrameBlocking(gotWord, body, bodyLen, sizeof(body), remaining)) return false;
        if (gotWord != wantWord) continue;   // stray frame, keep waiting
        if (bodyLen < 4) return fail(LD2402_ERR_BAD_REPLY);
        uint16_t status = body[0] | ((uint16_t)body[1] << 8);
        *gateMask = body[2] | ((uint16_t)body[3] << 8);
        *hadInterference = (status != 0);
        return true;
    }
    return fail(LD2402_ERR_TIMEOUT);
}

bool ld2402_save_parameters(uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);

    // Retried, because this is the one command whose failure is silent and
    // expensive. 0x00FD only *requests* the commit -- the module's main loop
    // does the erase and write -- so it can decline while busy with flash
    // work of its own, which is exactly the state it is in for a while after
    // a calibration. Observed: 32 threshold writes all landed and only the
    // commit was refused, leaving the values live in RAM and absent from
    // flash, to be lost at the next power cut.
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        sendCommand(0x00FD, nullptr, 0);
        if (waitAck(0x00FD, timeoutMs)) {
            // The module needs a moment to finish committing before config
            // mode is exited underneath it.
            vTaskDelay(pdMS_TO_TICKS(500));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    return false;
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
    if (!s.held) return fail(LD2402_ERR_BUSY);
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
    if (!s.held) return fail(LD2402_ERR_BUSY);
    sendCommand(0x00EE, nullptr, 0);
    if (!waitAck(0x00EE, timeoutMs)) return false;
    s_quiet_reason = "gain adjustment";
    s_autogain_running = true;
    // Auto-gain changes the front-end gain, so the energies the module
    // reports afterwards sit on a different scale than the cached thresholds
    // were read against. Same stale-comparison problem as calibration.
    invalidate_threshold_cache();
    return true;
}

bool ld2402_auto_gain_done(uint16_t timeoutMs) {
    UartSession s;
    if (!s.held) return fail(LD2402_ERR_BUSY);
    // The module pushes this unprompted (word 0x00F0, not a +0x0100 ACK) once
    // auto-gain finishes -- not a reply to a request we send.
    const bool done = waitEvent(0x00F0, timeoutMs);
    s_autogain_running = false;
    expect_module_busy_for(10);
    return done;
}
