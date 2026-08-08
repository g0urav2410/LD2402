// Driver for the Hi-Link HLK-LD2402 24GHz presence radar.
// Talks over a hardware UART at 115200 8N1 (module ships fixed at that rate).
// Self-contained: no dependency on any particular project, only Arduino Stream.
//
// Usage:
//   LD2402 radar;
//   radar.begin(Serial);       // any Stream: Serial, Serial1, SoftwareSerial...
//   loop() { radar.loop(); }   // parses whatever the module is streaming
//
// Live readings (presence/distance/energy) update continuously from whatever
// report mode the module is in - normal (text) or engineering (binary).
//
// Config/calibration calls are blocking (they wait for the module's ACK, up
// to a timeout) and are meant to be called rarely - from a setup screen or a
// one-off API call, not from the render loop. Wrap a batch of them in
// enableConfig()/endConfig().
#pragma once
#include <Arduino.h>

class LD2402 {
public:
    void begin(Stream &serial);
    // Reads and parses everything currently waiting on the stream. Use this
    // when the radar has the UART to itself.
    void loop();
    // Feed one byte at a time instead of calling loop(), for when something
    // else (e.g. a debug console) shares the same UART and needs to route
    // bytes between the two by content. midFrame() reports whether a binary
    // engineering frame is in progress, so a caller mid-frame knows to keep
    // routing bytes here regardless of their value.
    void feedByte(uint8_t b);
    bool midFrame() const { return _pstate != P_IDLE; }

    // ---- Live readings (updated by loop() from whatever is streaming) ----
    //
    // The engineering frame's first byte IS the enum the vendor manual
    // documents (Table 5-7): 0 nobody, 1 someone moving, 2 someone still.
    //
    // This library spent a long time believing otherwise. A firmware
    // reverse-engineering writeup decompiled the frame builder as
    //
    //     state = presence;                    // 0 or 1
    //     if (...) state += 0x10;
    //
    // and concluded 0x02 could never appear -- so moving/still was derived
    // instead, by comparing per-gate energies against cached thresholds.
    //
    // Logging the raw byte on a real module (fw v3.3.5, the same version
    // that writeup analysed) showed 0x00, 0x01 AND 0x02 in ordinary use, and
    // never 0x10 or 0x11. The writeup missed a code path. The manual was
    // right, and the module's own still detector -- a long-window CIC filter
    // sensitive enough for breathing -- is considerably better than the
    // energy-vs-threshold compare that had replaced it.
    //
    // ASCII mode carries no state byte at all, so a still person reads as
    // moving there. Call setEngineeringMode(true) for real still detection.
    //
    // If this is ever in doubt again: log the byte. Do not re-derive it from
    // a document, including this comment.
    // What the room is doing, as one value instead of three booleans you have
    // to combine yourself. This is the call to prefer.
    //
    // The three-boolean shape it replaces could contradict itself: isMoving()
    // looked only at gate energy and never checked presence, so a stray gate
    // over threshold with nobody detected produced "moving, but not present,
    // and not still" -- a state that cannot be true. One value cannot say
    // three things at once, which is the actual fix, not just nicer syntax.
    enum Activity : uint8_t { Absent = 0, Moving = 1, Still = 2 };

    // The raw state byte's documented values, used by presence()/activity().
    static constexpr uint8_t STATE_NOBODY = 0x00;
    static constexpr uint8_t STATE_MOVING = 0x01;
    static constexpr uint8_t STATE_STILL  = 0x02;
    Activity activity() const;

    bool presence() const;
    // Kept so existing sketches still compile. Both are now derived from
    // activity(), so they cannot disagree with it or with each other.
    bool isMoving() const { return activity() == Moving; }
    bool isStill()  const { return activity() == Still; }

    // Reads all 32 thresholds into a local cache, so the setAndSave* calls
    // can skip a flash write when a value is already what you're asking for.
    // Detection does not depend on this -- the module classifies moving/still
    // itself. Enters and leaves config mode; call it once at startup and
    // again after any calibration (which rewrites every threshold).
    bool cacheThresholds(uint16_t configTimeoutMs = 2500);
    bool haveThresholdCache() const { return _thresholdsValid; }
    // Throws the cache away, so activity() drops to its safe fallback until
    // cacheThresholds() runs again. Called automatically by startCalibration()
    // and startAutoGain(), both of which change what the cached numbers mean.
    void invalidateThresholdCache();
    // 0 once the sensor stops reporting -- see presence() in the .cpp for why
    // stale readings are cleared rather than held.
    uint16_t distanceCm() const { return connected() ? _distanceCm : 0; }
    bool haveEnergyGates() const { return _engineering; }
    // gate 0-15, near to far. NAN if no engineering data received yet.
    float triggerEnergyDb(uint8_t gate) const;
    float motionlessEnergyDb(uint8_t gate) const;
    unsigned long lastUpdateMs() const { return _lastUpdateMs; }
    bool connected() const { return _lastUpdateMs != 0 && millis() - _lastUpdateMs < 2000; }

    // Everything above, bundled into one call -- for the common case of just
    // wanting the current status without calling four separate getters.
    struct Reading {
        Activity activity;
        bool presence, moving, still, connected;
        uint16_t distanceCm;
    };
    Reading read() const {
        const Activity a = activity();
        return {a, presence(), a == Moving, a == Still, connected(), distanceCm()};
    }

    // ---- Diagnostics: raw byte flow, independent of frame parsing ----
    // Every byte the module sends increments this and refreshes lastByteMs(),
    // even garbage or a half-frame that never parses -- so these tell you
    // whether the module is transmitting *at all*, separately from whether
    // its output is being understood. If bytesReceived() keeps climbing but
    // no frame parses, it's a mode/parse problem; if it stops climbing, the
    // module itself went silent (power, wiring, or stuck in config mode).
    uint32_t bytesReceived() const { return _byteCount; }
    unsigned long lastByteMs() const { return _lastByteMs; }

    // Called repeatedly while this driver is blocked waiting on the module --
    // which is where essentially all of its time goes. On a single-threaded
    // board that is time in which nothing else in your sketch runs, so a
    // display sits frozen and animations stop unless this is used.
    //
    // Pass a short function that services whatever mustn't stall. It is called
    // very often, so keep it cheap (an early "has enough time passed?" test is
    // the usual shape), and do NOT call back into this driver from it -- a
    // command/response exchange is in progress the whole time.
    void onIdle(void (*fn)()) { _idle = fn; }

    // ---- Config / calibration (blocking, call rarely) ----
    bool enableConfig(uint16_t timeoutMs = 1000);
    bool endConfig(uint16_t timeoutMs = 1000);

    bool readFirmwareVersion(String &out, uint16_t timeoutMs = 1000);
    bool readSerialNumber(String &out, uint16_t timeoutMs = 1000);

    // true = binary engineering frames (presence+distance+32 energy gates)
    // false = plain text "OFF" / "distance : NN" (the module's own default)
    bool setOutputMode(bool engineering, uint16_t timeoutMs = 1000);

    // Convenience: wraps enableConfig()/setOutputMode()/endConfig() in one
    // call, own session -- same pattern as the setAndSave* functions below.
    //
    // You do not normally need to call this. loop() enables engineering mode
    // on its own and restores it if the module restarts, because still
    // detection does not work without it. Call it with `false` only if you
    // deliberately want the lighter text format and can live without
    // moving/still.
    //
    // configTimeoutMs is passed to enableConfig(); default matches every
    // other convenience function here, but a caller doing frequent quick
    // checks (like a periodic recovery poll) can pass a much shorter one.
    bool setEngineeringMode(bool on, uint16_t configTimeoutMs = 2500);

    bool setMaxDistanceMeters(float meters, uint16_t timeoutMs = 1000);   // 0.7-10.0m
    bool readMaxDistanceMeters(float &meters, uint16_t timeoutMs = 1000);
    bool setDisappearDelaySec(uint16_t seconds, uint16_t timeoutMs = 1000);
    bool readDisappearDelaySec(uint16_t &seconds, uint16_t timeoutMs = 1000);
    // Convenience: set + persist in one call, own config session -- same
    // pattern as the threshold convenience methods below. Exiting config mode
    // commits the change to flash by itself, no explicit save needed.
    // saveTimeoutMs is accepted for API compatibility but unused.
    bool setAndSaveMaxDistanceMeters(float meters, uint16_t configTimeoutMs = 2500, uint16_t saveTimeoutMs = 3000);
    bool setAndSaveDisappearDelaySec(uint16_t seconds, uint16_t configTimeoutMs = 2500, uint16_t saveTimeoutMs = 3000);

    bool setTriggerThresholdDb(uint8_t gate, float db, uint16_t timeoutMs = 1000);   // gate 0-15
    bool readTriggerThresholdDb(uint8_t gate, float &db, uint16_t timeoutMs = 1000);
    bool setMotionlessThresholdDb(uint8_t gate, float db, uint16_t timeoutMs = 1000);    // gate 0-15
    bool readMotionlessThresholdDb(uint8_t gate, float &db, uint16_t timeoutMs = 1000);

    // ---- Convenience: set + persist in one call, own config session ----
    // Callers who just want "change this threshold and have it stick" can use
    // these instead of manually wrapping setXThresholdDb() in enableConfig()/
    // endConfig(). Persistence is automatic: exiting config mode commits the
    // change to flash by itself -- confirmed by direct protocol-level testing
    // (write every gate 0-15's trigger and motionless threshold plus max
    // distance and disappear delay, exit config with no save command sent at
    // all, power-cycle the sensor, read every value back: all 34/34
    // persisted). No explicit save command is needed for any parameter,
    // including gate 15's motionless threshold, which an earlier, narrower
    // test had mistakenly flagged as needing special handling.
    // saveTimeoutMs is accepted for API compatibility but unused.
    bool setAndSaveTriggerThresholdDb(uint8_t gate, float db, uint16_t configTimeoutMs = 2500, uint16_t saveTimeoutMs = 3000);
    bool setAndSaveMotionlessThresholdDb(uint8_t gate, float db, uint16_t configTimeoutMs = 2500, uint16_t saveTimeoutMs = 3000);

    // Writes all 16 trigger + all 16 motionless thresholds and persists them
    // all in one enableConfig()/endConfig() session -- no explicit save
    // command needed, see above. Pass nullptr for either array to skip it
    // entirely. saveTimeoutMs is accepted for API compatibility but unused.
    bool saveAllThresholds(const float triggerDb[16], const float motionlessDb[16],
                            uint16_t configTimeoutMs = 2500, uint16_t saveTimeoutMs = 8000);

    // 0 = not run, 1 = clear, 2 = interference present
    bool readPowerInterference(uint8_t &status, uint16_t timeoutMs = 1000);

    // Auto threshold calibration. factor 1-20ish (module multiplies by 10 internally).
    bool startCalibration(uint8_t triggerFactor = 3, uint8_t holdFactor = 3, uint8_t microFactor = 3, uint16_t timeoutMs = 1000);
    bool calibrationProgress(uint8_t &percent, uint16_t timeoutMs = 1000); // 100 = done

    // REMOVED -- readCalibrationInterference() never worked.
    //
    // It sent command 0x0014 believing that reported whether a human moved
    // through the room during calibration. A full disassembly of firmware
    // v3.3.5 shows 0x0014 is a plain ping: it returns two constant 0x0000
    // bytes and looks at nothing. So the call could only ever answer "no
    // interference, no gates affected", whatever actually happened -- an
    // always-clear result that read as a real all-clear.
    //
    // There is no replacement, because the module does not appear to expose
    // this at all. Keeping the room genuinely empty during calibration has to
    // be the operator's job. (This matters more than it sounds: a calibration
    // run with someone still in the room measures *them*, and thresholds set
    // above that leave the sensor detecting nothing.)

    bool saveParameters(uint16_t timeoutMs = 1000); // firmware >= 3.3.2

    // Restarts the module (command 0x00EF, confirmed in the v3.3.5
    // disassembly). It exits config mode on its own, so nothing needs to be
    // wrapped around this; the module goes quiet for a moment and then starts
    // streaming again.
    //
    // Note that a restart drops it back to text output -- output mode lives in
    // RAM, not flash -- so call setEngineeringMode(true) again afterwards if
    // you were using it.
    //
    // There is deliberately no factoryReset(): the protocol has no such
    // command. Restoring defaults means writing the values back yourself.
    bool reboot(uint16_t timeoutMs = 1000);

    bool startAutoGain(uint16_t timeoutMs = 1000);           // firmware >= 3.3.5
    bool autoGainDone(uint16_t timeoutMs = 3000);             // waits for the module's completion push

    bool readParameterRaw(uint16_t id, uint32_t &value, uint16_t timeoutMs = 1000);
    bool setParameterRaw(uint16_t id, uint32_t value, uint16_t timeoutMs = 1000);

    // ---- Why the last config/calibration call failed ----
    //
    // Every call above returns a plain bool, and for a long time that was all
    // a caller got. But `false` covers several situations needing completely
    // different responses -- retry, fix the value you passed, go and check the
    // wiring -- and the driver knew which and threw it away.
    //
    // errno-style on purpose, rather than changing forty return types: the
    // reason is recorded at the few places a call can actually fail, so every
    // function gains it without its signature changing and without forcing any
    // caller to care. Only meaningful straight after a call returned false --
    // it is not cleared on success.
    enum Error : uint8_t {
        ERR_NONE = 0,
        ERR_TIMEOUT,        // the module did not answer in time
        ERR_REFUSED,        // the module answered, and said no
        ERR_BAD_REPLY,      // an answer arrived, too short or malformed to use
        ERR_BAD_ARG,        // the value asked for is out of range
        ERR_NOT_CONNECTED,  // no bytes at all from the module -- power/wiring
    };
    Error lastError() const { return _lastErr; }
    // "timeout", "refused", "bad_reply", "bad_arg", "not_connected", "ok".
    static const char *errorString(Error e);
    const char *lastErrorString() const { return errorString(_lastErr); }

private:
    Error _lastErr = ERR_NONE;
    bool fail(Error e) { _lastErr = e; return false; }
    // Counts bytes read during one exchange, so a failed wait can tell "the
    // module said nothing at all" from "the module is talking but never sent
    // the answer". Those look identical from a bool and want opposite
    // reactions -- check the cable, versus try again.
    uint32_t _exchangeBytes = 0;

    Stream *_serial = nullptr;

    // --- streaming parse state (text or engineering-binary, module picks one) ---
    enum ParseState { P_IDLE, P_HDR2, P_HDR3, P_HDR4, P_LEN1, P_LEN2, P_BODY, P_FOOT1, P_FOOT2, P_FOOT3, P_FOOT4 };
    ParseState _pstate = P_IDLE;
    uint16_t _bodyLen = 0, _bodyIdx = 0;
    uint8_t _body[200];
    String _lineBuf;

    uint8_t _state = 0;   // raw state byte -- see presence() above

    // Engineering mode is on unless a sketch explicitly turns it off: it is
    // what still detection needs, and the module drops it whenever it
    // restarts. loop() re-applies it -- see maintainEngineeringMode().
    bool _wantEngineering = true;
    unsigned long _lastEngAttemptMs = 0;
    void maintainEngineeringMode();

    // Per-gate thresholds mirrored locally so the setAndSave* calls can skip
    // a flash write when nothing changed, without a UART round trip to check.
    // Kept in step by the read/set threshold calls; only counts as usable
    // once every gate in both sets has a real value, so that "not read yet"
    // is never mistaken for "already correct" and a real write dropped.
    float _triggerTh[16] = {0};
    float _motionlessTh[16] = {0};
    uint16_t _trigSeen = 0, _motSeen = 0;
    bool _thresholdsValid = false;
    void noteThresholdProgress() {
        if (_trigSeen == 0xFFFF && _motSeen == 0xFFFF) _thresholdsValid = true;
    }
    uint16_t _distanceCm = 0;
    bool _engineering = false;
    uint32_t _energy[32] = {0};
    unsigned long _lastUpdateMs = 0;
    uint32_t _byteCount = 0;        // every byte ever fed, diagnostic
    unsigned long _lastByteMs = 0;  // millis() of the last byte fed
    void (*_idle)() = nullptr;      // see onIdle()

    // yield() plus the caller's idle hook -- used everywhere this driver waits.
    void idleWait() { if (_idle) _idle(); yield(); }
    // delay(), broken up so the idle hook still runs during it.
    void idleDelay(uint16_t ms) {
        unsigned long start = millis();
        while ((uint16_t)(millis() - start) < ms) { idleWait(); }
    }

    void handleTextByte(uint8_t b);
    void handleTextLine(String line);
    void handleEngineeringFrame(const uint8_t *body, uint16_t len);

    // --- command/ACK framing (FD FC FB FA ... 04 03 02 01) ---
    void sendCommand(uint16_t word, const uint8_t *value, uint16_t valueLen);
    // Reads one byte and updates bytesReceived()/lastByteMs() -- same
    // diagnostics feedByte() updates for the streaming parser, so those stay
    // accurate during config-mode command exchanges too, not just streaming.
    uint8_t readByteTracked();
    // Blocks reading raw bytes (bypassing the streaming parser) until a full
    // FD-FC-FB-FA frame arrives or timeoutMs elapses. Returns the frame's
    // word field and body (word+status, or word+status+extra).
    bool readFrameBlocking(uint16_t &word, uint8_t *body, uint16_t &bodyLen, uint16_t maxBody, uint16_t timeoutMs);
    // Waits for the ACK to `word` (module echoes word+0x0100). extra/extraLen
    // receive whatever follows the 2-byte status, if requested.
    bool waitAck(uint16_t word, uint16_t timeoutMs, uint8_t *extra = nullptr, uint16_t extraCap = 0, uint16_t *extraLen = nullptr);
    // Waits for an unsolicited frame carrying exactly `word` (not +0x0100) -
    // used for the auto-gain completion push.
    bool waitEvent(uint16_t word, uint16_t timeoutMs);
};
