#include "LD2402.h"
#include <math.h>
#include <string.h>

// ---- byte/frame constants from the HLK-LD2402 manual v1.08, section 5 ----
static const uint8_t CMD_HDR[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FOOT[4] = {0x04, 0x03, 0x02, 0x01};
static const uint8_t ENG_HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t ENG_FOOT[4] = {0xF8, 0xF7, 0xF6, 0xF5};

static float dbFromRaw(uint32_t raw) {
    return raw > 0 ? 10.0f * log10f((float)raw) : 0.0f;
}
static uint32_t rawFromDb(float db) {
    return (uint32_t)roundf(powf(10.0f, db / 10.0f));
}

void LD2402::begin(Stream &serial) {
    _serial = &serial;
    _lineBuf.reserve(32);
}

// ---------------------------------------------------------------- streaming

void LD2402::loop() {
    if (!_serial) return;
    while (_serial->available()) {
        feedByte((uint8_t)_serial->read());
    }
}

void LD2402::feedByte(uint8_t b) {
    _byteCount++;
    _lastByteMs = millis();
    switch (_pstate) {
        case P_IDLE:
            if (b == ENG_HDR[0]) _pstate = P_HDR2;
            else handleTextByte(b);
            return;
        case P_HDR2: _pstate = (b == ENG_HDR[1]) ? P_HDR3 : P_IDLE; return;
        case P_HDR3: _pstate = (b == ENG_HDR[2]) ? P_HDR4 : P_IDLE; return;
        case P_HDR4: _pstate = (b == ENG_HDR[3]) ? P_LEN1 : P_IDLE; return;
        case P_LEN1: _bodyLen = b; _pstate = P_LEN2; return;
        case P_LEN2:
            _bodyLen |= ((uint16_t)b << 8);
            _bodyIdx = 0;
            if (_bodyLen == 0 || _bodyLen > sizeof(_body)) { _pstate = P_IDLE; return; }
            _pstate = P_BODY;
            return;
        case P_BODY:
            _body[_bodyIdx++] = b;
            if (_bodyIdx >= _bodyLen) _pstate = P_FOOT1;
            return;
        // Footer bytes are checked against ENG_FOOT, not just counted -- with
        // nothing wired in, a floating RX pin picks up real electrical noise
        // (especially this close to a WiFi radio), and an unvalidated footer
        // let noise that happened to produce a plausible header+length get
        // accepted as a genuine frame, occasionally reporting fake presence.
        // Four states for four footer bytes (F8 F7 F6 F5) -- P_FOOT1 receives
        // the first one, P_FOOT4 the last.
        case P_FOOT1: _pstate = (b == ENG_FOOT[0]) ? P_FOOT2 : P_IDLE; return;
        case P_FOOT2: _pstate = (b == ENG_FOOT[1]) ? P_FOOT3 : P_IDLE; return;
        case P_FOOT3: _pstate = (b == ENG_FOOT[2]) ? P_FOOT4 : P_IDLE; return;
        case P_FOOT4:
            _pstate = P_IDLE;
            if (b == ENG_FOOT[3]) handleEngineeringFrame(_body, _bodyLen);
            return;
    }
}

void LD2402::handleTextByte(uint8_t b) {
    if (b == '\n') {
        handleTextLine(_lineBuf);
        _lineBuf = "";
        return;
    }
    if (b == '\r') return;
    if (_lineBuf.length() < 40) _lineBuf += (char)b;
    else _lineBuf = ""; // garbage/overlong line, drop it
}

void LD2402::handleTextLine(String line) {
    line.trim();
    if (line.length() == 0) return;
    if (line == "OFF") {
        _state = 0;
        _distanceCm = 0;
        _engineering = false;
        _lastUpdateMs = millis();
        return;
    }
    int colon = line.indexOf(':');
    if (line.startsWith("distance") && colon >= 0) {
        _state = 1;
        _distanceCm = (uint16_t)line.substring(colon + 1).toInt();
        _engineering = false;
        _lastUpdateMs = millis();
    }
}

void LD2402::handleEngineeringFrame(const uint8_t *body, uint16_t len) {
    if (len < 3) return;
    _state = body[0];
    _distanceCm = body[1] | ((uint16_t)body[2] << 8);
    _engineering = true;
    if (len >= 3 + 32 * 4) {
        for (uint8_t i = 0; i < 32; i++) {
            uint16_t off = 3 + i * 4;
            _energy[i] = (uint32_t)body[off] | ((uint32_t)body[off + 1] << 8) |
                         ((uint32_t)body[off + 2] << 16) | ((uint32_t)body[off + 3] << 24);
        }
    }
    _lastUpdateMs = millis();
}

float LD2402::triggerEnergyDb(uint8_t gate) const {
    return gate < 16 ? dbFromRaw(_energy[gate]) : NAN;
}
float LD2402::motionlessEnergyDb(uint8_t gate) const {
    return gate < 16 ? dbFromRaw(_energy[16 + gate]) : NAN;
}

// ----------------------------------------------------------------- commands

void LD2402::sendCommand(uint16_t word, const uint8_t *value, uint16_t valueLen) {
    if (!_serial) return;
    uint16_t len = 2 + valueLen;
    _serial->write(CMD_HDR, 4);
    _serial->write((uint8_t)(len & 0xFF));
    _serial->write((uint8_t)(len >> 8));
    _serial->write((uint8_t)(word & 0xFF));
    _serial->write((uint8_t)(word >> 8));
    if (valueLen) _serial->write(value, valueLen);
    _serial->write(CMD_FOOT, 4);
}

// Reads one byte and updates the same "have I heard from the sensor"
// diagnostics feedByte() does for the streaming parser -- readFrameBlocking()
// used to bypass those entirely, so a caller checking lastByteMs()/
// connected() during/right after a config-mode session (which can run
// several seconds for a bulk save) would see it as stale even though the
// sensor was actively exchanging ACKs the whole time.
uint8_t LD2402::readByteTracked() {
    uint8_t b = (uint8_t)_serial->read();
    _byteCount++;
    _lastByteMs = millis();
    return b;
}

bool LD2402::readFrameBlocking(uint16_t &word, uint8_t *body, uint16_t &bodyLen, uint16_t maxBody, uint16_t timeoutMs) {
    unsigned long start = millis();
    uint8_t match = 0; // how many of CMD_HDR matched so far
    while ((uint16_t)(millis() - start) < timeoutMs) {
        if (!_serial->available()) { idleWait(); continue; }  // let the ESP service WiFi/watchdog, and the caller's onIdle hook run
        uint8_t b = readByteTracked();
        if (match < 4) {
            if (b == CMD_HDR[match]) match++;
            else match = (b == CMD_HDR[0]) ? 1 : 0;
            // Only fall through once the 4th header byte (FA) has JUST matched.
            // The bug this fixes: the old code did `continue` here even on the
            // 4th match, so the next loop iteration read a fresh byte -- the
            // frame's first length byte -- and discarded it, then read the
            // length from the following two bytes. Every response came back
            // with a garbage length and was rejected, so no read/ACK ever
            // parsed. `b` here is FA, the last header byte, NOT a length byte;
            // the length is read fresh below.
            if (match < 4) continue;
        }
        // header complete; next 2 bytes = length
        uint8_t lenBuf[2];
        uint8_t got = 0;
        while (got < 2 && (uint16_t)(millis() - start) < timeoutMs) {
            if (_serial->available()) lenBuf[got++] = readByteTracked();
            else idleWait();
        }
        if (got < 2) return false;
        uint16_t len = lenBuf[0] | ((uint16_t)lenBuf[1] << 8);
        // A nonsense length means those four "header" bytes were noise that
        // happened to look like FD FC FB FA, not the start of a real frame.
        // Resync and keep hunting within the time that's left, rather than
        // giving up on the whole exchange -- one corrupted byte used to abort
        // a wait that still had most of its budget remaining, turning a
        // recoverable glitch into a failed command.
        if (len < 2 || len > maxBody) { match = 0; continue; }
        uint16_t idx = 0;
        while (idx < len && (uint16_t)(millis() - start) < timeoutMs) {
            if (_serial->available()) body[idx++] = readByteTracked();
            else idleWait();
        }
        if (idx < len) return false;
        // Check the footer rather than just counting four bytes past the body.
        // The streaming parser already validates its own footer, for exactly
        // this reason: an unvalidated one lets noise that produced a plausible
        // header and length be accepted as a genuine frame.
        uint8_t foot = 0;
        bool footOk = true;
        while (foot < 4 && (uint16_t)(millis() - start) < timeoutMs) {
            if (_serial->available()) {
                if (readByteTracked() != CMD_FOOT[foot]) footOk = false;
                foot++;
            } else idleWait();
        }
        if (foot < 4) return false;
        if (!footOk) { match = 0; continue; }   // garbled -- resync, don't accept
        word = body[0] | ((uint16_t)body[1] << 8);
        bodyLen = len - 2;
        for (uint16_t i = 0; i < bodyLen; i++) body[i] = body[i + 2];
        return true;
    }
    return false;
}

bool LD2402::waitAck(uint16_t word, uint16_t timeoutMs, uint8_t *extra, uint16_t extraCap, uint16_t *extraLen) {
    uint16_t wantWord = word + 0x0100;
    unsigned long start = millis();
    while ((uint16_t)(millis() - start) < timeoutMs) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = timeoutMs - (uint16_t)(millis() - start);
        if (!readFrameBlocking(gotWord, _body, bodyLen, sizeof(_body), remaining)) return false;
        if (gotWord != wantWord) continue; // stray frame (e.g. an unsolicited event), keep waiting
        if (bodyLen < 2) return false;
        uint16_t status = _body[0] | ((uint16_t)_body[1] << 8);
        if (status != 0) return false;
        if (extra && extraLen) {
            uint16_t n = bodyLen - 2;
            if (n > extraCap) n = extraCap;
            memcpy(extra, _body + 2, n);
            *extraLen = n;
        }
        return true;
    }
    return false;
}

bool LD2402::waitEvent(uint16_t word, uint16_t timeoutMs) {
    unsigned long start = millis();
    while ((uint16_t)(millis() - start) < timeoutMs) {
        uint16_t gotWord, bodyLen;
        uint16_t remaining = timeoutMs - (uint16_t)(millis() - start);
        if (!readFrameBlocking(gotWord, _body, bodyLen, sizeof(_body), remaining)) return false;
        if (gotWord == word) return true;
    }
    return false;
}

bool LD2402::enableConfig(uint16_t timeoutMs) {
    // Deadline-based hammering, not a fixed 3 tries. Breaking into config mode
    // is easy when the module is idle or text-streaming, but HARD while it's
    // firehosing engineering frames (~128 bytes every 165ms): the enable ACK
    // is buried in that flood and a few quick attempts miss it. The moment the
    // module does accept the command it STOPS streaming, so the very next
    // attempt sees a clean, easily-caught ACK. So we keep re-sending (each
    // send is harmless -- a redundant enable while already in config just
    // ACKs) with a short per-attempt wait until we catch one or run out of
    // budget. Callers give this a generous timeout (a couple of seconds) for
    // the engineering-mode case; it returns as soon as it succeeds.
    const uint8_t val[2] = {0x01, 0x00};
    const unsigned long deadline = millis() + timeoutMs;
    do {
        while (_serial->available()) readByteTracked(); // drop buffered stream bytes
        sendCommand(0x00FF, val, 2);
        if (waitAck(0x00FF, 250)) return true;
    } while ((long)(deadline - millis()) > 0);
    return false;
}

bool LD2402::endConfig(uint16_t timeoutMs) {
    // Retries, unlike a naive single send: exiting config mode is what resumes
    // the module's data stream, so a single missed ACK here (garbled by line
    // noise or the host's own TX traffic on a shared UART) would leave the
    // module parked in config mode -- silent forever, looking exactly like a
    // dead sensor. Better to send it a few times; a redundant exit while
    // already streaming is simply ignored by the module.
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        while (_serial->available()) readByteTracked(); // drop stale bytes first
        sendCommand(0x00FE, nullptr, 0);
        if (waitAck(0x00FE, timeoutMs)) return true;
        idleDelay(100);
    }
    return false;
}

bool LD2402::readFirmwareVersion(String &out, uint16_t timeoutMs) {
    sendCommand(0x0000, nullptr, 0);
    uint8_t extra[64];
    uint16_t n = 0;
    if (!waitAck(0x0000, timeoutMs, extra, sizeof(extra), &n) || n < 2) return false;
    uint16_t verLen = extra[0] | ((uint16_t)extra[1] << 8);
    if (verLen > n - 2) verLen = n - 2;
    out = "";
    for (uint16_t i = 0; i < verLen; i++) out += (char)extra[2 + i];
    return true;
}

bool LD2402::readSerialNumber(String &out, uint16_t timeoutMs) {
    sendCommand(0x0011, nullptr, 0);
    uint8_t extra[40];
    uint16_t n = 0;
    if (!waitAck(0x0011, timeoutMs, extra, sizeof(extra), &n) || n < 2) return false;
    uint16_t snLen = extra[0] | ((uint16_t)extra[1] << 8);
    if (snLen > n - 2) snLen = n - 2;
    out = "";
    for (uint16_t i = 0; i < snLen; i++) out += (char)extra[2 + i];
    return true;
}

bool LD2402::setOutputMode(bool engineering, uint16_t timeoutMs) {
    uint32_t mode = engineering ? 0x00000004 : 0x00000064;
    // The value field is SIX bytes, not four: a 2-byte "command value" of
    // 0x0000 followed by the 4-byte mode (manual 5.2.8 -- the send example has
    // data length 0x0008 = 2-byte command word + 2-byte command value +
    // 4-byte parameter). Sending only the 4-byte mode made a malformed frame
    // the module either rejected or misread, which corrupted its output and
    // left it silent after the switch -- the whole reason engineering mode
    // "killed" the stream. ESPHome's working component sends the same 6 bytes.
    uint8_t val[6] = {0x00, 0x00,
                      (uint8_t)mode, (uint8_t)(mode >> 8),
                      (uint8_t)(mode >> 16), (uint8_t)(mode >> 24)};
    sendCommand(0x0012, val, 6);
    // Takes effect once endConfig() is called - the module won't stream
    // engineering frames while still in config mode.
    return waitAck(0x0012, timeoutMs);
}

bool LD2402::setEngineeringMode(bool on, uint16_t configTimeoutMs) {
    // Bail out instead of pressing on: with config mode never entered,
    // every command below waits out its full timeout for an ACK that
    // cannot come, and endConfig() then burns three more retries. The
    // result was already correct, just several seconds late.
    if (!enableConfig(configTimeoutMs)) return false;
    bool ok = setOutputMode(on);
    endConfig(configTimeoutMs);   // same timeout as enableConfig() -- a caller
                                   // asking for a fast enable almost certainly
                                   // wants a fast exit too, not a 1000ms default
    return ok;
}

bool LD2402::readParameterRaw(uint16_t id, uint32_t &value, uint16_t timeoutMs) {
    uint8_t val[2] = {(uint8_t)(id & 0xFF), (uint8_t)(id >> 8)};
    sendCommand(0x0008, val, 2);
    uint8_t extra[4];
    uint16_t n = 0;
    if (!waitAck(0x0008, timeoutMs, extra, sizeof(extra), &n) || n < 4) return false;
    value = (uint32_t)extra[0] | ((uint32_t)extra[1] << 8) | ((uint32_t)extra[2] << 16) | ((uint32_t)extra[3] << 24);
    return true;
}

bool LD2402::setParameterRaw(uint16_t id, uint32_t value, uint16_t timeoutMs) {
    uint8_t val[6] = {
        (uint8_t)(id & 0xFF), (uint8_t)(id >> 8),
        (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    sendCommand(0x0007, val, 6);
    return waitAck(0x0007, timeoutMs);
}

bool LD2402::setMaxDistanceMeters(float meters, uint16_t timeoutMs) {
    int v = (int)roundf(meters * 10.0f);
    if (v < 7) v = 7;
    if (v > 100) v = 100;
    return setParameterRaw(0x0001, (uint32_t)v, timeoutMs);
}
bool LD2402::readMaxDistanceMeters(float &meters, uint16_t timeoutMs) {
    uint32_t raw;
    if (!readParameterRaw(0x0001, raw, timeoutMs)) return false;
    meters = raw / 10.0f;
    return true;
}

bool LD2402::setDisappearDelaySec(uint16_t seconds, uint16_t timeoutMs) {
    return setParameterRaw(0x0004, seconds, timeoutMs);
}
bool LD2402::readDisappearDelaySec(uint16_t &seconds, uint16_t timeoutMs) {
    uint32_t raw;
    if (!readParameterRaw(0x0004, raw, timeoutMs)) return false;
    seconds = (uint16_t)raw;
    return true;
}

bool LD2402::setAndSaveMaxDistanceMeters(float meters, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    // endConfig() does NOT commit to flash. Verified on hardware: values set
    // through here read back correctly and then revert after the module is
    // power-cycled. 0x00FD (saveParameters) is the only thing that persists
    // them, and it merely *requests* the save -- the module's main loop does
    // the erase and program afterwards -- so it waits before returning
    // rather than letting endConfig race the write.
    // Bail out instead of pressing on: with config mode never entered,
    // every command below waits out its full timeout for an ACK that
    // cannot come, and endConfig() then burns three more retries. The
    // result was already correct, just several seconds late.
    if (!enableConfig(configTimeoutMs)) return false;
    bool ok = setMaxDistanceMeters(meters);
    if (ok) ok = saveParameters(saveTimeoutMs);
    endConfig(configTimeoutMs);
    return ok;
}

bool LD2402::setAndSaveDisappearDelaySec(uint16_t seconds, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    // Bail out instead of pressing on: with config mode never entered,
    // every command below waits out its full timeout for an ACK that
    // cannot come, and endConfig() then burns three more retries. The
    // result was already correct, just several seconds late.
    if (!enableConfig(configTimeoutMs)) return false;
    bool ok = setDisappearDelaySec(seconds);
    if (ok) ok = saveParameters(saveTimeoutMs);
    endConfig(configTimeoutMs);
    return ok;
}

bool LD2402::setTriggerThresholdDb(uint8_t gate, float db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    if (!setParameterRaw(0x0010 + gate, rawFromDb(db), timeoutMs)) return false;
    _triggerTh[gate] = db;   // mirror into the classifier cache -- see isMoving()
    return true;
}
bool LD2402::readTriggerThresholdDb(uint8_t gate, float &db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    uint32_t raw;
    if (!readParameterRaw(0x0010 + gate, raw, timeoutMs)) return false;
    db = dbFromRaw(raw);
    _triggerTh[gate] = db;
    _trigSeen |= (uint16_t)1u << gate;
    noteThresholdProgress();
    return true;
}
bool LD2402::setMotionlessThresholdDb(uint8_t gate, float db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    if (!setParameterRaw(0x0030 + gate, rawFromDb(db), timeoutMs)) return false;
    _motionlessTh[gate] = db;
    return true;
}
bool LD2402::readMotionlessThresholdDb(uint8_t gate, float &db, uint16_t timeoutMs) {
    if (gate > 15) return false;
    uint32_t raw;
    if (!readParameterRaw(0x0030 + gate, raw, timeoutMs)) return false;
    db = dbFromRaw(raw);
    _motionlessTh[gate] = db;
    _motSeen |= (uint16_t)1u << gate;
    noteThresholdProgress();
    return true;
}

bool LD2402::setAndSaveTriggerThresholdDb(uint8_t gate, float db, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (gate > 15) return false;
    // Bail out instead of pressing on: with config mode never entered,
    // every command below waits out its full timeout for an ACK that
    // cannot come, and endConfig() then burns three more retries. The
    // result was already correct, just several seconds late.
    if (!enableConfig(configTimeoutMs)) return false;
    bool ok = setTriggerThresholdDb(gate, db);
    if (ok) ok = saveParameters(saveTimeoutMs);
    endConfig(configTimeoutMs);
    return ok;
}

bool LD2402::setAndSaveMotionlessThresholdDb(uint8_t gate, float db, uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    if (gate > 15) return false;
    // Bail out instead of pressing on: with config mode never entered,
    // every command below waits out its full timeout for an ACK that
    // cannot come, and endConfig() then burns three more retries. The
    // result was already correct, just several seconds late.
    if (!enableConfig(configTimeoutMs)) return false;
    bool ok = setMotionlessThresholdDb(gate, db);
    if (ok) ok = saveParameters(saveTimeoutMs);
    endConfig(configTimeoutMs);
    return ok;
}

bool LD2402::saveAllThresholds(const float triggerDb[16], const float motionlessDb[16],
                                uint16_t configTimeoutMs, uint16_t saveTimeoutMs) {
    // Bail out instead of pressing on: with config mode never entered,
    // every command below waits out its full timeout for an ACK that
    // cannot come, and endConfig() then burns three more retries. The
    // result was already correct, just several seconds late.
    if (!enableConfig(configTimeoutMs)) return false;
    bool ok = true;
    if (triggerDb) for (uint8_t i = 0; i < 16; i++) ok &= setTriggerThresholdDb(i, triggerDb[i]);
    if (motionlessDb) for (uint8_t i = 0; i < 16; i++) ok &= setMotionlessThresholdDb(i, motionlessDb[i]);
    if (ok) ok = saveParameters(saveTimeoutMs);   // see setAndSaveMaxDistanceMeters
    endConfig(configTimeoutMs);
    return ok;
}

bool LD2402::readPowerInterference(uint8_t &status, uint16_t timeoutMs) {
    uint32_t raw;
    if (!readParameterRaw(0x0005, raw, timeoutMs)) return false;
    status = (uint8_t)raw;
    return true;
}

bool LD2402::startCalibration(uint8_t triggerFactor, uint8_t holdFactor, uint8_t microFactor, uint16_t timeoutMs) {
    uint16_t trig = (uint16_t)triggerFactor * 10, hold = (uint16_t)holdFactor * 10, micro = (uint16_t)microFactor * 10;
    uint8_t val[6] = {
        (uint8_t)trig, (uint8_t)(trig >> 8),
        (uint8_t)hold, (uint8_t)(hold >> 8),
        (uint8_t)micro, (uint8_t)(micro >> 8)};
    sendCommand(0x0009, val, 6);
    if (!waitAck(0x0009, timeoutMs)) return false;
    // Calibration rewrites all 32 thresholds inside the module, which makes
    // every cached copy here wrong. Nothing used to invalidate them, so
    // activity() went on comparing fresh energies against pre-calibration
    // thresholds indefinitely -- silently, and with no way for a caller to
    // notice. Dropping the cache costs a re-read and is obviously correct;
    // the alternative was a classifier quietly running on stale numbers.
    invalidateThresholdCache();
    return true;
}

bool LD2402::calibrationProgress(uint8_t &percent, uint16_t timeoutMs) {
    sendCommand(0x000A, nullptr, 0);
    uint8_t extra[2];
    uint16_t n = 0;
    if (!waitAck(0x000A, timeoutMs, extra, sizeof(extra), &n) || n < 2) return false;
    percent = extra[0]; // percentage fits in one byte (0-100), extra[1] is always 0
    return true;
}

// Presence comes from the module's own detection chain, which is the
// authoritative answer to "is anyone there" -- but it is only meaningful
// while the module is still talking to us. Held readings are worse than no
// readings here: unplug the sensor mid-run and the obvious sketch,
// `if (radar.presence())`, would go on believing someone is in the room
// forever, with nothing on screen to suggest otherwise.
bool LD2402::presence() const {
    return connected() && (_state & 0x0F) != 0;
}

// One decision, made once, so the three answers cannot disagree.
//
// Absent is decided first and absolutely: with nobody detected, "moving" is
// not a question worth asking. Only then do the gate energies pick between
// Moving and Still -- mirroring the module's own peak selection, scanning
// gates 1..15 for any whose motion energy clears that gate's trigger
// threshold. Gate 0 is skipped for the reason the module skips it: its
// threshold is never evaluated, so including it would let near-field clutter
// set a flag no configuration change could clear.
//
// Without a threshold cache there is nothing to compare against, so anyone
// present is reported as Moving -- the safer of the two wrong answers, since
// a missed still-person is a sensor that looks broken while a missed movement
// is merely a sensor that looks insensitive.
LD2402::Activity LD2402::activity() const {
    if (!presence()) return Absent;
    if (!_engineering || !_thresholdsValid) return Moving;
    for (uint8_t g = 1; g < 16; g++) {
        const float e = triggerEnergyDb(g);
        if (!isnan(e) && e > _triggerTh[g]) return Moving;
    }
    return Still;
}

void LD2402::invalidateThresholdCache() {
    _thresholdsValid = false;
    _trigSeen = 0;
    _motSeen = 0;
}

bool LD2402::cacheThresholds(uint16_t configTimeoutMs) {
    if (!enableConfig(configTimeoutMs)) return false;
    // Start from nothing rather than topping up whatever is already there. A
    // refresh after calibration must not be able to leave a mix of new and
    // stale values behind if some of the 32 reads fail partway through.
    invalidateThresholdCache();
    float db;
    for (uint8_t g = 0; g < 16; g++) {
        readTriggerThresholdDb(g, db);
        readMotionlessThresholdDb(g, db);
    }
    endConfig(configTimeoutMs);
    return _thresholdsValid;
}

bool LD2402::saveParameters(uint16_t timeoutMs) {
    sendCommand(0x00FD, nullptr, 0);
    if (!waitAck(0x00FD, timeoutMs)) return false;
    idleDelay(500); // module needs time to commit to flash before config mode is exited
    return true;
}

bool LD2402::startAutoGain(uint16_t timeoutMs) {
    sendCommand(0x00EE, nullptr, 0);
    if (!waitAck(0x00EE, timeoutMs)) return false;
    // Auto-gain changes the front-end gain, so every energy number the module
    // reports afterwards sits on a different scale than the cached thresholds
    // were read against. Same stale-comparison problem as calibration.
    invalidateThresholdCache();
    return true;
}

bool LD2402::reboot(uint16_t timeoutMs) {
    sendCommand(0x00EF, nullptr, 0);
    if (!waitAck(0x00EF, timeoutMs)) return false;
    // Everything cached here described the module that just went away: its
    // thresholds are re-read from flash on boot, and its output mode is not
    // persistent at all, so it comes back in text mode with no energy gates.
    invalidateThresholdCache();
    _engineering = false;
    _state = 0;
    _distanceCm = 0;
    return true;
}

bool LD2402::autoGainDone(uint16_t timeoutMs) {
    // The module pushes this unprompted (word 0x00F0, not a +0x0100 ACK)
    // once auto-gain finishes - it is not a reply to a request we send.
    return waitEvent(0x00F0, timeoutMs);
}
