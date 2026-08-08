// BasicPresence -- prints presence, movement and distance to the Serial
// Monitor. Wire the sensor's T/R pins to a spare hardware UART (see
// README.md "Wiring"), then flash this.
//
// Note the sensor gets its OWN serial port, separate from the USB one you
// read the output on. Sharing them means your debug prints get sent to the
// sensor and the sensor's frames get printed as garbage.
#include <LD2402.h>

LD2402 radar;

void setup() {
    Serial.begin(115200);    // USB, for reading this output
    Serial1.begin(115200);   // the sensor
    radar.begin(Serial1);

    // Nothing else needed. loop() switches the sensor into the output mode
    // that reports moving vs still, and puts it back if the sensor restarts.
}

void loop() {
    radar.loop();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 500) return;
    lastPrint = millis();

    if (!radar.connected()) {
        Serial.println("(no data from sensor yet)");
        return;
    }
    if (!radar.haveEnergyGates()) {
        // Still switching the sensor over -- takes a few seconds after boot.
        Serial.println("(waiting for the sensor to switch output mode)");
        return;
    }

    switch (radar.activity()) {
        case LD2402::Absent:
            Serial.println("empty");
            break;
        case LD2402::Moving:
            Serial.printf("moving, %d cm\n", radar.distanceCm());
            break;
        case LD2402::Still:
            Serial.printf("still, %d cm\n", radar.distanceCm());
            break;
    }
}
