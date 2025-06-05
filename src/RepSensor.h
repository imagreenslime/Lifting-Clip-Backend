#ifndef REP_SENSOR_H
#define REP_SENSOR_H

struct SetData {
    int repCount;
    float durations[100];
    float roms[100];
    float tempos[100][3];
};

const SetData* getSets();  // returns pointer to sets array
int getSetCount();
bool isRecording();
int getRepCount();

void setupSensors();
void updateSensor();
void startCalibration();
void startRecording();
void stopRecording();
void saveCurrentSet();


#endif