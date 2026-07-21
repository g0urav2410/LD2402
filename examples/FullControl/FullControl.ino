// FullControl -- switches the sensor into engineering mode (distance + all
// 32 energy gates), runs an auto-calibration, and prints everything to the
// Serial Monitor. This is the "see everything, tune everything" example --
// what the sensor's own PC tool and Home Assistant integrations expose.
#include <LD2402.h>

LD2402 radar;

void setup() {
    Serial.begin(115200);
    radar.begin(Serial);
    delay(500);

    if (!radar.enableConfig()) {
        Serial.println("Sensor not responding -- check wiring/power.");
        return;
    }

    String fw;
    if (radar.readFirmwareVersion(fw)) Serial.println("Firmware: " + fw);

    // Full data: presence + distance + all 32 energy gates, not just on/off.
    radar.setOutputMode(true);

    // A few basic settings, applied live (not yet saved to the sensor's flash).
    radar.setMaxDistanceMeters(5.0f);
    radar.setDisappearDelaySec(5);

    Serial.println("Calibrating -- stay out of the sensor's view...");
    radar.startCalibration(/*trigger*/3, /*hold*/3, /*micro*/3);
    uint8_t pct = 0;
    while (radar.calibrationProgress(pct) && pct < 100) {
        Serial.printf("  %d%%\n", pct);
        delay(500);
    }
    Serial.println(pct >= 100 ? "Calibration done." : "Calibration did not finish.");

    // Only commits to the sensor's own flash if you want it to survive a
    // power cycle -- separate on purpose from "apply now".
    // radar.saveParameters();

    radar.endConfig();   // must exit config mode before data streaming resumes
}

void loop() {
    radar.loop();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 1000) return;
    lastPrint = millis();

    if (!radar.haveEnergyGates()) return;   // no engineering frame yet

    Serial.printf("presence=%d moving=%d distance=%dcm\n",
                   radar.presence(), radar.isMoving(), radar.distanceCm());
    Serial.print("motion (dB):  ");
    for (int i = 0; i < 16; i++) Serial.printf("%.0f ", radar.motionEnergyDb(i));
    Serial.println();
    Serial.print("micro  (dB):  ");
    for (int i = 0; i < 16; i++) Serial.printf("%.0f ", radar.microEnergyDb(i));
    Serial.println();
}
