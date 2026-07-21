// BasicPresence -- minimal example: prints presence + distance to the
// Serial Monitor. Wire the sensor's T/R pins to the board's hardware UART
// (see README.md "Wiring"), then flash this.
//
// The module has the UART to itself here, so loop() is all that's needed --
// no manual byte routing. See the SharedUART example if something else
// (a debug console, another sensor) needs the same wire.
#include <LD2402.h>

LD2402 radar;

void setup() {
    Serial.begin(115200);
    radar.begin(Serial);
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
    if (radar.presence()) {
        Serial.printf("Presence: %s, %d cm\n",
                       radar.isMoving() ? "moving" : "still",
                       radar.distanceCm());
    } else {
        Serial.println("Presence: none");
    }
}
