#include "RepBLE.h"
#include "RepSensor.h"

NimBLECharacteristic* repCharacteristic;
NimBLECharacteristic* commandCharacteristic;
NimBLECharacteristic* setCharacteristic;

bool deviceConnected = false;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define REP_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SET_CHARACTERISTIC_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define COMMAND_CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

void sendSetSummary(){
    if (!deviceConnected) return;
    Serial.print("sending set data");
    
    int setsCount = getSetCount();
    const SetData* sets = getSets();
    const SetData& latestSet = sets[setsCount - 1];
    Serial.printf("Sending Set %d with %d reps\n", setsCount, latestSet.repCount);
    String json = "[";
    for (int j = 0; j < latestSet.repCount; j++) {
    json += String("{\"id\":") + millis() +
        ",\"set\":" + String(setsCount) +
        ",\"rep\":" + String(j + 1) +
        ",\"dur\":" + String(latestSet.durations[j], 2) +
        ",\"rom\":" + String(latestSet.roms[j], 2) +
         ",\"tempo\":[" + String(latestSet.tempos[j][0], 2) + "," + 
                        String(latestSet.tempos[j][1], 2) + "," + 
                        String(latestSet.tempos[j][2], 2) + "]}";
        if (j != latestSet.repCount - 1) json += ",";
    }
    json += "]";
        Serial.println(json);
    setCharacteristic->setValue(json.c_str());
    setCharacteristic->notify();
    Serial.println(". set data finished sending");
}

//C:\Users\shane\Documents\PlatformIO\Projects\ESP32-Tutorial-2\.pio\libdeps\upesy_wroom\NimBLE-Arduino\src
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("✅ Connected to phone");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("❌ Client disconnected");
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks: public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    std::string value = characteristic->getValue();
    Serial.print("got command: ");
    Serial.println(value.c_str());
    if (value == "START_RECORDING"){
        startRecording();
    } else if (value == "STOP_RECORDING"){
        stopRecording();
        saveCurrentSet();
        sendSetSummary();
    } else if (value == "CALIBRATE"){

    }
  }
};

void setupBLE() {
    NimBLEDevice::init("RepCounter");

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new MyServerCallbacks());

    Serial.println("set server callbacks");

    NimBLEService* service = server->createService(SERVICE_UUID);
    repCharacteristic = service->createCharacteristic(
        REP_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    repCharacteristic->setValue("0");

    commandCharacteristic= service->createCharacteristic(
        COMMAND_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    commandCharacteristic->setCallbacks(new CommandCallbacks());
    Serial.println("set command callbacks");
    
    setCharacteristic = service->createCharacteristic(
        SET_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setName("RepCounter");
    advertising->enableScanResponse(true);
    advertising->start();
    Serial.println("Advertising has started");
}

void notifyData(){
    if (!deviceConnected || !isRecording()) return;

    repCharacteristic->setValue(String(getRepCount()).c_str());
    repCharacteristic->notify();
}

void handleBLE(){
    
}

