// LD2402 -- Arduino/PlatformIO driver for the Hi-Link HLK-LD2402 24GHz
// presence radar (moving / micro-motion / static human presence).
//
// https://github.com/g0urav2410/LD2402  -- see README.md for wiring, the full
// protocol reference, and worked examples.
//
// Talks over a hardware UART at 115200 8N1 (the module's baud rate is fixed
// in firmware, not configurable). Self-contained: no dependency on any
// particular project, only Arduino's Stream interface, so it works equally
// on Serial, Serial1/Serial2 (ESP32), or a SoftwareSerial instance.
//
// Quick start:
//   LD2402 radar;
//   void setup() { Serial.begin(115200); radar.begin(Serial); }
//   void loop()  { radar.loop();
//                  if (radar.presence()) Serial.println(radar.distanceCm()); }
//
// Live readings (presence/distance/energy) update continuously from whatever
// report mode the module is in -- normal (text) or engineering (binary, adds
// the 32 energy gates). See setOutputMode().
//
// Config/calibration calls are blocking (they wait for the module's ACK, up
// to a timeout) and are meant to be called rarely -- from a setup routine or
// a one-off menu action, not every loop(). Wrap a batch of them in
// enableConfig()/endConfig() so the module only has to be told once.
//
// Sharing the UART with something else (e.g. a debug console)? Don't call
// loop() -- see feedByte()/midFrame() and the "SharedUART" note in README.md.
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
    bool presence() const { return _result != 0; }
    bool isMoving() const { return _result == 1; }
    bool isStill() const { return _result == 2; }
    uint16_t distanceCm() const { return _distanceCm; }
    bool haveEnergyGates() const { return _engineering; }
    // gate 0-15, near to far. NAN if no engineering data received yet.
    float motionEnergyDb(uint8_t gate) const;
    float microEnergyDb(uint8_t gate) const;
    unsigned long lastUpdateMs() const { return _lastUpdateMs; }
    bool connected() const { return _lastUpdateMs != 0 && millis() - _lastUpdateMs < 2000; }

    // ---- Config / calibration (blocking, call rarely) ----
    bool enableConfig(uint16_t timeoutMs = 1000);
    bool endConfig(uint16_t timeoutMs = 1000);

    bool readFirmwareVersion(String &out, uint16_t timeoutMs = 1000);
    bool readSerialNumber(String &out, uint16_t timeoutMs = 1000);

    // true = binary engineering frames (presence+distance+32 energy gates)
    // false = plain text "OFF" / "distance : NN" (factory default)
    bool setOutputMode(bool engineering, uint16_t timeoutMs = 1000);

    bool setMaxDistanceMeters(float meters, uint16_t timeoutMs = 1000);   // 0.7-10.0m
    bool readMaxDistanceMeters(float &meters, uint16_t timeoutMs = 1000);
    bool setDisappearDelaySec(uint16_t seconds, uint16_t timeoutMs = 1000);
    bool readDisappearDelaySec(uint16_t &seconds, uint16_t timeoutMs = 1000);

    bool setMotionThresholdDb(uint8_t gate, float db, uint16_t timeoutMs = 1000);   // gate 0-15
    bool readMotionThresholdDb(uint8_t gate, float &db, uint16_t timeoutMs = 1000);
    bool setMicroThresholdDb(uint8_t gate, float db, uint16_t timeoutMs = 1000);    // gate 0-15
    bool readMicroThresholdDb(uint8_t gate, float &db, uint16_t timeoutMs = 1000);

    // 0 = not run, 1 = clear, 2 = interference present
    bool readPowerInterference(uint8_t &status, uint16_t timeoutMs = 1000);

    // Auto threshold calibration. factor 1-20ish (module multiplies by 10 internally).
    bool startCalibration(uint8_t triggerFactor = 3, uint8_t holdFactor = 3, uint8_t microFactor = 3, uint16_t timeoutMs = 1000);
    bool calibrationProgress(uint8_t &percent, uint16_t timeoutMs = 1000); // 100 = done

    bool saveParameters(uint16_t timeoutMs = 1000); // firmware >= 3.3.2

    bool startAutoGain(uint16_t timeoutMs = 1000);           // firmware >= 3.3.5
    bool autoGainDone(uint16_t timeoutMs = 3000);             // waits for the module's completion push

    bool readParameterRaw(uint16_t id, uint32_t &value, uint16_t timeoutMs = 1000);
    bool setParameterRaw(uint16_t id, uint32_t value, uint16_t timeoutMs = 1000);

private:
    Stream *_serial = nullptr;

    // --- streaming parse state (text or engineering-binary, module picks one) ---
    enum ParseState { P_IDLE, P_HDR2, P_HDR3, P_HDR4, P_LEN1, P_LEN2, P_BODY, P_FOOT1, P_FOOT2, P_FOOT3, P_FOOT4 };
    ParseState _pstate = P_IDLE;
    uint16_t _bodyLen = 0, _bodyIdx = 0;
    uint8_t _body[200];
    String _lineBuf;

    uint8_t _result = 0;       // 0 none, 1 moving, 2 still
    uint16_t _distanceCm = 0;
    bool _engineering = false;
    uint32_t _energy[32] = {0};
    unsigned long _lastUpdateMs = 0;

    void handleTextByte(uint8_t b);
    void handleTextLine(String line);
    void handleEngineeringFrame(const uint8_t *body, uint16_t len);

    // --- command/ACK framing (FD FC FB FA ... 04 03 02 01) ---
    void sendCommand(uint16_t word, const uint8_t *value, uint16_t valueLen);
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
