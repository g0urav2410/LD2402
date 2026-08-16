#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

// Driver for the Hi-Link HLK-LD2402 24GHz presence radar, ported from the
// standalone Arduino driver (github.com/g0urav2410/LD2402) to ESP-IDF/FreeRTOS.
//
// The protocol, the frame layout, the gate math and every quirk/workaround
// comment below are carried over unchanged from that driver -- they describe
// the sensor's actual firmware behaviour, not anything specific to Arduino or
// ESP-IDF. What changed is the concurrency model, and it's a genuine
// simplification, not a like-for-like port:
//
// On the ESP8266, this same UART also had to serve a debug console and share
// CPU time with everything else in one loop(), so the original driver carried
// an onIdle() callback threaded through every blocking wait, purely so a
// multi-second config exchange didn't freeze the display. Here the radar has
// a task of its own with sole ownership of UART0 (freed up by the console
// living on native USB instead -- see ../HARDWARE.md). A task is simply
// *allowed* to block. onIdle() and the byte-by-byte cooperative polling it
// existed for are gone; blocking config calls just block.
//
// What still needs enforcing, and is enforced by a mutex rather than a
// polling flag: only one thing may be mid-exchange on the UART at a time.
// ld2402_task()'s own streaming read loop and every config/*() call below
// content for the same mutex, so a config call from another task cleanly
// pauses the streaming parse for its duration rather than racing it.

#ifdef __cplusplus
extern "C" {
#endif

// What the room is doing, as one value. Matches the Arduino driver's
// LD2402::Activity so both drivers describe the sensor the same way.
typedef enum {
    LD2402_ABSENT = 0,
    LD2402_MOVING = 1,
    LD2402_STILL  = 2,
} ld2402_activity_t;

// Why the last config/calibration call failed.
//
// Every function in the "Config / calibration" section below returns a plain
// bool, and for a long time that was all a caller got. But `false` covers at
// least five situations that need completely different responses -- wait and
// retry, retry now, fix the value you passed, go and check the wiring -- and
// the driver knew which one it was and threw it away. A settings screen could
// only ever say "Failed".
//
// Deliberately errno-style rather than changing forty return types: the reason
// is recorded at the handful of places a call can actually fail (the mutexes,
// the ACK wait, argument checks), so every function gets it without its
// signature changing and without any caller being forced to care.
typedef enum {
    LD2402_OK = 0,
    LD2402_ERR_BUSY,           // another config session or exchange is in progress
    LD2402_ERR_TIMEOUT,        // the module did not answer in time
    LD2402_ERR_REFUSED,        // the module answered, and said no
    LD2402_ERR_BAD_REPLY,      // an answer arrived, too short or malformed to use
    LD2402_ERR_BAD_ARG,        // the value asked for is out of range
    LD2402_ERR_NOT_CONNECTED,  // no bytes at all from the module -- power/wiring
} ld2402_err_t;

// Why the most recent failed call failed.
//
// Only meaningful immediately after a call returned false -- like errno, it is
// not cleared on success, so reading it after something worked tells you about
// an older failure. Config calls are serialised by the session mutex, so the
// value belongs to the call you just made.
ld2402_err_t ld2402_last_error(void);

// Short, stable, lowercase identifier for a reason: "busy", "timeout",
// "refused", "bad_reply", "bad_arg", "not_connected", "ok". Suitable for
// putting straight into a JSON error field.
const char *ld2402_err_str(ld2402_err_t err);

// Notification hook for things the sensor does on its own -- going silent,
// coming back, having its engineering mode restored after it rebooted itself.
// Optional; leave it null and these are only ESP_LOG lines. The string is
// valid only for the duration of the call.
typedef void (*ld2402_event_cb_t)(const char *message);

// Where the module is wired, supplied at init rather than compiled in.
typedef struct {
    uart_port_t uart_port;
    int pin_tx;                  // ESP TX -> module R
    int pin_rx;                  // ESP RX <- module T
    ld2402_event_cb_t event_cb;  // optional
    // UART receive buffer, bytes. 0 uses the 1KB default. Raise it if this
    // board does anything that stalls tasks for long stretches -- a flash
    // write with the cache disabled halts this driver's task, and only this
    // buffer stands between that and lost frames (~775 bytes/s streaming).
    int rx_buf_size;
} ld2402_config_t;

typedef struct {
    // All three come straight from the module's own classification (the
    // engineering frame's state byte: 0 nobody, 1 moving, 2 still), so they
    // are mutually exclusive by construction.
    //
    // In ASCII mode there is no state byte, so a still person reads as
    // moving. Keep engineering mode on -- the driver does by default.
    bool presence;         // any detection, moving or still
    bool moving;
    bool still;
    // The same answer as the three booleans above, as one value. Prefer it:
    // three booleans describing one thing can be read in combinations that
    // cannot happen, and callers end up re-deriving the rules. These are kept
    // because the HTTP API, MQTT discovery and the app all publish them
    // individually, but they are all decided in one place -- see the note in
    // ld2402.cpp -- so they cannot disagree with this or with each other.
    ld2402_activity_t activity;
    bool connected;        // a frame has arrived recently (see radar.cpp)
    uint16_t distance_cm;
    bool engineering;      // true once a binary engineering frame has parsed
    float trigger_db[16];   // gate 0-15, near to far; NAN until engineering data arrives
    float motionless_db[16];
    uint32_t bytes_received;   // diagnostic: total bytes seen, garbage included
    int64_t last_byte_us;      // esp_timer_get_time() of the last byte, any kind
    int64_t last_update_us;    // esp_timer_get_time() of the last successfully parsed reading
} ld2402_reading_t;

// Installs the UART0 driver on the pins in pins.h and starts the task that
// keeps `ld2402_reading_t` current. Call once.
esp_err_t ld2402_init(const ld2402_config_t *cfg);

// Cached reads -- no UART, no config mode, safe to call as often as you like.
//
// The slow equivalents below each cost a config-mode session and a round trip
// per value (32 of them for the whole threshold set). On a host whose web
// server handles one request at a time, that is seconds during which nothing
// else gets answered, which is exactly what makes a settings screen feel
// stuck. These serve the copies the driver already maintains.
//
// false means the value has not been read from the module yet -- do the slow
// read once, and these are populated as a side effect.
bool ld2402_get_cached_thresholds(float trigger_db[16], float motionless_db[16]);

// How far through a bulk threshold write the driver is, for a caller wanting
// to show progress. ld2402_save_all_thresholds() is a single call taking ten
// seconds or so, so without this the only honest options are a spinner or a
// bar that jumps 0 to 100. Safe to call from another task while the write runs.
void ld2402_bulk_write_progress(int *done, int *total);
bool ld2402_get_cached_max_distance_m(float *meters);
bool ld2402_get_cached_disappear_delay_s(uint16_t *seconds);

// The engineering frame's first byte as received: 0 nobody, 1 moving, 2 still.
// Everything in the frame is decoded into ld2402_reading_t already, so this is
// only for looking at what the module sent rather than what this driver made
// of it -- which is how a wrong, disassembly-backed claim about this byte was
// finally caught. See the note above s_state in ld2402.cpp.
uint8_t ld2402_debug_raw_state(void);

// Every distinct state byte seen since boot, as a bitmask: bit N set means
// byte N has arrived at least once. Cleared only by a reboot.
//
// The instantaneous byte above answers "what is it doing now"; this answers
// "has this module ever reported still at all", which is the question when
// stillness seems never to happen. One is a sample and can miss; the other
// cannot. Bytes above 31 are folded into bit 31 rather than dropped -- an
// unexpected value showing up is itself the finding.
uint32_t ld2402_debug_state_seen_mask(void);

// Thread-safe snapshot of the current reading. Cheap, never blocks on the
// sensor -- reads a cached struct under a short mutex hold.
void ld2402_get_reading(ld2402_reading_t *out);

// ---- Config / calibration ----
// All of these are blocking (they wait for the module's ACK, up to
// timeout_ms) and contend for the UART mutex with the streaming reader, so
// they pause live readings for their duration. Meant to be called rarely --
// from an HTTP handler off a settings screen, not from any hot path. Safe to
// call from any task; do not call from ld2402_task itself.

bool ld2402_enable_config(uint16_t timeout_ms);
bool ld2402_end_config(uint16_t timeout_ms);

// buf/cap: caller-owned buffer, no String -- matches the rest of this port.
bool ld2402_read_firmware_version(char *buf, size_t cap, uint16_t timeout_ms);
bool ld2402_read_serial_number(char *buf, size_t cap, uint16_t timeout_ms);

// true = binary engineering frames (presence+distance+32 energy gates)
// false = plain text "OFF" / "distance : NN" (factory default)
bool ld2402_set_output_mode(bool engineering, uint16_t timeout_ms);
// Convenience: own enableConfig()/endConfig() session.
bool ld2402_set_engineering_mode(bool on, uint16_t config_timeout_ms);

bool ld2402_set_max_distance_m(float meters, uint16_t timeout_ms);          // 0.7-10.0m
bool ld2402_read_max_distance_m(float *meters, uint16_t timeout_ms);
bool ld2402_set_disappear_delay_s(uint16_t seconds, uint16_t timeout_ms);
bool ld2402_read_disappear_delay_s(uint16_t *seconds, uint16_t timeout_ms);
// Convenience: set + persist, own config session.
bool ld2402_set_and_save_max_distance_m(float meters, uint16_t config_timeout_ms, uint16_t save_timeout_ms);
bool ld2402_set_and_save_disappear_delay_s(uint16_t seconds, uint16_t config_timeout_ms, uint16_t save_timeout_ms);

// Batched parameter access. The module's read and write commands each take N
// records per frame (manual 5.2.6 / 5.2.7), and this driver used to send one
// per command -- so reading all 32 thresholds meant 32 command/ACK round
// trips, which is most of why that took seconds. These split the request
// across as many frames as the module's reported buffer allows.
//
// Caller must already hold a config session.
bool ld2402_read_parameters_raw(const uint16_t *ids, uint32_t *values,
                                 uint8_t count, uint16_t timeout_ms);
bool ld2402_set_parameters_raw(const uint16_t *ids, const uint32_t *values,
                                uint8_t count, uint16_t timeout_ms);

// All 32 thresholds in two frames rather than 32 exchanges. Either output may
// be null -- the driver's cache is filled regardless, which is the only thing
// the boot-time priming needs. Caller must already hold a config session.
bool ld2402_read_all_thresholds(float trigger_db[16], float motionless_db[16],
                                 uint16_t timeout_ms);

bool ld2402_set_trigger_threshold_db(uint8_t gate, float db, uint16_t timeout_ms);   // gate 0-15
bool ld2402_read_trigger_threshold_db(uint8_t gate, float *db, uint16_t timeout_ms);
bool ld2402_set_motionless_threshold_db(uint8_t gate, float db, uint16_t timeout_ms);    // gate 0-15
bool ld2402_read_motionless_threshold_db(uint8_t gate, float *db, uint16_t timeout_ms);

// Convenience: set + persist in one call, own config session. Exiting config
// mode commits every parameter to flash by itself -- confirmed via direct
// protocol testing (all 34 writable parameters, including gate 15's
// motionless threshold, survive a real power cycle with no explicit save
// command ever sent). save_timeout_ms is accepted for API compatibility but
// unused.
bool ld2402_set_and_save_trigger_threshold_db(uint8_t gate, float db, uint16_t config_timeout_ms, uint16_t save_timeout_ms);
bool ld2402_set_and_save_motionless_threshold_db(uint8_t gate, float db, uint16_t config_timeout_ms, uint16_t save_timeout_ms);

// Writes all 16 motion + all 16 motionless thresholds and persists them in one
// session (gate 15's motionless threshold still needs its own procedure regardless
// -- see above). Pass NULL for either array to skip it.
bool ld2402_save_all_thresholds(const float trigger_db[16], const float motionless_db[16],
                                uint16_t config_timeout_ms, uint16_t save_timeout_ms);

// The same write, with a say in whether it reaches the module's flash.
//
// `commit == false` leaves the values in the module's RAM: they take effect
// immediately and are gone on its next power cycle. That is what a live
// control wants -- something you can drag, watch, and change your mind about
// without spending a flash erase per adjustment on a part with a finite
// number of them and no documented wear levelling.
//
// `commit == true` is exactly ld2402_save_all_thresholds().
bool ld2402_write_all_thresholds(const float trigger_db[16], const float motionless_db[16],
                                  bool commit, uint16_t config_timeout_ms,
                                  uint16_t save_timeout_ms);

bool ld2402_read_power_interference(uint8_t *status, uint16_t timeout_ms);   // 0 not run, 1 clear, 2 interference

// Auto threshold calibration. factor 1-20ish (module multiplies by 10 internally).
bool ld2402_start_calibration(uint8_t trigger_factor, uint8_t hold_factor, uint8_t micro_factor, uint16_t timeout_ms);
bool ld2402_calibration_progress(uint8_t *percent, uint16_t timeout_ms);   // 100 = done
// gate_mask bit N set = interference seen at gate N (~0.7m per gate). Call once
// calibration reaches 100%.
bool ld2402_read_calibration_interference(bool *had_interference, uint16_t *gate_mask, uint16_t timeout_ms);

bool ld2402_save_parameters(uint16_t timeout_ms);   // firmware >= 3.3.2

// Restarts the sensor module itself (not the ESP). It comes back in text
// mode -- output mode is not persistent -- but the driver's watchdog restores
// engineering mode and re-primes the threshold cache on its own.
bool ld2402_reboot(uint16_t timeout_ms);

bool ld2402_start_auto_gain(uint16_t timeout_ms);            // firmware >= 3.3.5
bool ld2402_auto_gain_done(uint16_t timeout_ms);              // waits for the module's completion push

bool ld2402_read_parameter_raw(uint16_t id, uint32_t *value, uint16_t timeout_ms);
bool ld2402_set_parameter_raw(uint16_t id, uint32_t value, uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif
