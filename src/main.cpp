#include <Arduino.h>
#include "RepSensor.h"
#include "RepBLE.h"

void setup() {
    Serial.begin(9600);
    setupSensors();
    setupBLE();
}

void loop() {
    if (isRecording()){
        updateSensor(); // Handles sensor processing and rep detection
        handleBLE();    // Handles BLE notifications and command listening
    }

}