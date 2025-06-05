#include "RepSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Fusion.h>

// Pin Definitions
#define BUTTON_PIN 26
#define TOF_INT_PIN 5

// new
bool recording = false;

// Sensor objects
Adafruit_MPU6050 mpu;
FusionAhrs ahrs;
VL53L1X tof;

// kalman filter variables
float est_velocity = 0.0;
float velocityError = 1.0;
const float process_noise = 1.0;
const float measurement_noise = .5;

// ToF buffer variables
const int BUFFER_SIZE = 4;
float tofBuffer[BUFFER_SIZE] = {0};
bool bufferFilled = false;
int tofIndex = 0;

// Timinng and Velocity
unsigned long lastLoopTime = 0;
float previousAccelerationZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float previousHeightInches = 0.0;
float velocitySmoothed = 0.0;
float velocityRaw = 0.0;
float velocityZ = 0.0;
const float filterAlpha = .2;

// counting rep variables
enum RepState { REST, DESCENDING, ASCENDING };
RepState repState = REST;
unsigned long repStartTime = 0;
const unsigned long repTimeoutMs = 800;

int repCount = 0;
const int MAX_REPS = 100;
float repDurations[MAX_REPS];
float repROM[MAX_REPS];
float repTempos[MAX_REPS][3];
int repIndex = 0;

// Set tracking

const int MAX_SETS = 10;
SetData sets[MAX_SETS];
int setIndex = 0;

// range of motion variables

float currentRepMin = 9999.0f;
float currentRepMax = -9999.0f;
const float userHeightInches = 64.0;
const float ROM_ratio = 0.30f; 
const float romThresholdSquat = userHeightInches * ROM_ratio;

// tempo analysis variables

unsigned long tStartDescending = 0;
unsigned long tStartBottom = 0;
unsigned long tStartAscending = 0;

float timeEccentric = 0;
float timePause = 0;
float timeConcentric = 0;
bool downCaptured = false;
bool bottomCaptured = false;

// Expoential moving averages
float jerkEma = 0.0f;
float accelEma = 0.0f;

// get functions
const SetData* getSets(){
    return sets;
}

int getRepCount(){
    return repCount;
}
int getSetCount(){
    return setIndex;
}

bool isRecording(){
    return recording;
}


void calibrateAcceleration(){
    Serial.println("calibrating");
    sensors_event_t a, g, temp;

    float totalAx = 0, totalAy = 0, totalAz = 0;

    for (int i = 0; i < 200; i++){
        mpu.getEvent(&a, &g, &temp);
        totalAx += a.acceleration.x;
        totalAy += a.acceleration.y;
        totalAz += a.acceleration.z;
        delay(5);
    }
    accelBiasX = totalAx / 200;
    accelBiasY = totalAy / 200;
    accelBiasZ = totalAz / 200;
}

void setupSensors(){

    Wire.begin(18, 19); // SDA: 18, SCL: 19
    pinMode(TOF_INT_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    if (!tof.init()) {
    Serial.println("Failed to find VL53L1X sensor");
    while (1);
    }

    Serial.println("TOF is ready");
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(100000);  // ms
    tof.setROISize(8,8);
    tof.startContinuous(100);
    previousHeightInches = tof.read() * (0.0393701);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            Serial.print(".");
            delay(10);
        }
    }
    
    Serial.println("MPU ready");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    FusionAhrsInitialise(&ahrs);
    delay(2000);
    calibrateAcceleration();
    lastLoopTime = millis();
}

void startRecording(){
    recording = true;
    repIndex = 0;
    repCount = 0;
}

void stopRecording(){
    recording = false;
    
}

float getToFAverage(){
    int count = bufferFilled ? BUFFER_SIZE : tofIndex;
    float sum = 0.0;
    for (int i = 0; i < count; i++){
        sum += tofBuffer[i];
    }
    return (count > 0) ? (sum / count) : 0.0f;
}

void updateSensor(){
    if (!tof.dataReady()) return;
    float heightMm = tof.read();
    float heightInches = heightMm * 0.0393701;
    tofBuffer[tofIndex++] = heightInches;
    
    if (tofIndex >= BUFFER_SIZE) {
        tofIndex = 0;
        bufferFilled = true;
    }
    
    float averageHeight = getToFAverage();
    unsigned long now = millis();
    float dt = (now - lastLoopTime)/1000.0f;
    lastLoopTime = now;

    float deltaHeight = averageHeight - previousHeightInches;
       if (abs(deltaHeight) > 0.1 && abs(deltaHeight) < 12.0) {
        velocityRaw = deltaHeight/dt;
    }

    velocitySmoothed = filterAlpha * velocityRaw + (1 - filterAlpha) * velocitySmoothed;
    previousHeightInches = averageHeight;

    currentRepMin = min(currentRepMin, averageHeight);
    currentRepMax = max(currentRepMax, averageHeight);

    if (currentRepMax > userHeightInches) currentRepMax = userHeightInches;
    if (currentRepMin < 12) currentRepMin = 12;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    FusionVector gyro = {g.gyro.x * 180.0f / PI, g.gyro.y * 180.0f / PI, g.gyro.z * 180.0f / PI};
    FusionVector accel = {a.acceleration.x - accelBiasX, a.acceleration.y - accelBiasY, a.acceleration.z - accelBiasZ}; 

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, dt);
    FusionMatrix rot = FusionQuaternionToMatrix(FusionAhrsGetQuaternion(&ahrs));
    FusionVector worldAccel = FusionMatrixMultiplyVector(rot, accel);

    float zAccel = worldAccel.array[2];
    float jerk = (zAccel-previousAccelerationZ)/dt; 
    previousAccelerationZ = zAccel;
    
    accelEma = filterAlpha * zAccel + (1.0f - filterAlpha) * accelEma;
    jerkEma = filterAlpha * jerk + (1.0f - filterAlpha) * jerkEma;

    // Kalman filter fusion
    float velocityMps = velocitySmoothed * 0.0254f; // convert inches/sec to m/s
    velocityError += process_noise;
    float kalmanGain = velocityError / (velocityError + measurement_noise);
    est_velocity += kalmanGain * (velocityMps - est_velocity);
    velocityError *= (1.0f - kalmanGain);
    float fusedVelocity = est_velocity * 39.3701f;
    Serial.print("Velocity: "); Serial.print(fusedVelocity); 
    // count reps
    bool validROM = (currentRepMax - currentRepMin) > romThresholdSquat;
    bool atTop = ((abs(averageHeight - currentRepMax) < 8.0f)) || (averageHeight > userHeightInches - 12.0f);

    Serial.print(". TOTAL D: "); Serial.print(currentRepMax - currentRepMin);
    Serial.print(". D FROM S: "); Serial.print(averageHeight - currentRepMax);
    Serial.print(". HEIGHT: "); Serial.println(averageHeight);
    if (fusedVelocity < -1.0f && abs(jerkEma) > .75f){
        if (repState == REST && !downCaptured) {
            repState = DESCENDING;
            downCaptured = true;
            repStartTime = now;
            tStartDescending = now;
            Serial.println("Start DOWN");
        }

    }
    if (fusedVelocity > 1.0f && abs(jerkEma) > .75f){
        if (repState != ASCENDING && validROM){
            repState = ASCENDING;
            tStartAscending = now;
            Serial.println("Start UP");
        }
    }

    if (repState == DESCENDING && !bottomCaptured && abs(jerkEma) < 0.75f && validROM) {
        tStartBottom = now;
        bottomCaptured = true;
        fusedVelocity = velocitySmoothed = est_velocity = jerkEma = accelEma = 0;
        Serial.println("Bottom Pause Detected");
    }

    else if ((now - repStartTime > repTimeoutMs) && atTop && validROM){
        repCount++;
        float rom = currentRepMax - currentRepMin;
        currentRepMax = -9999.9f;
        currentRepMin = 9999.9f;
            
        if (bottomCaptured){
            timeEccentric = (tStartBottom - tStartDescending) / 1000.0f;
            timePause = (tStartAscending - tStartBottom) / 1000.0f;
            timeConcentric = (now - tStartAscending) / 1000.0f;
        } else {
            timeEccentric = (tStartAscending - tStartDescending) / 1000.0f;
            timePause = 0.0f;
            timeConcentric = (now - tStartAscending) / 1000.0f;
        }

        tStartDescending = tStartBottom = tStartAscending = 0;
        downCaptured = bottomCaptured = false;

        float repDuration = (now - repStartTime) / 1000.0f;
        repStartTime = now;

        repDurations[repIndex] = repDuration; 
        repROM[repIndex] = rom;
        repTempos[repIndex][0] = timeEccentric;
        repTempos[repIndex][1] = timePause;
        repTempos[repIndex][2] = timeConcentric;
        repIndex++;

        Serial.print("Rep "); Serial.print(repCount);
        Serial.print(", duration: "); Serial.print(repDurations[repIndex - 1]);
        Serial.print("s, ROM: "); Serial.print(rom); Serial.println(" inches");

        Serial.print("Tempo: ");
        Serial.print(timeEccentric); Serial.print("-");
        Serial.print(timePause); Serial.print("-");
        Serial.println(timeConcentric);

        repState = REST;
        timeEccentric = timePause = timeConcentric = 0;
        fusedVelocity = velocitySmoothed = est_velocity = jerkEma = accelEma = 0;
        Serial.println("Back to REST");
        }
}

void saveCurrentSet(){
    if (setIndex >= MAX_SETS) return;
    sets[setIndex].repCount = repIndex;
    for (int i = 0; i < repIndex; i++){
        sets[setIndex].durations[i] = repDurations[i];
        sets[setIndex].roms[i] = repROM[i];
        sets[setIndex].tempos[i][0] = repTempos[i][0];
        sets[setIndex].tempos[i][1] = repTempos[i][1];
        sets[setIndex].tempos[i][2] = repTempos[i][2];
    }
    setIndex++;
    repIndex = 0;
    repCount = 0;
    Serial.println("Set Saved");
}

void printAllSets() {
    Serial.println("=== All Sets Summary ===");
    for (int i = 0; i < setIndex; i++) {
        Serial.print("Set "); Serial.print(i + 1);
        Serial.print(" - Reps: "); Serial.println(sets[i].repCount);
        for (int j = 0; j < sets[i].repCount; j++) {
            Serial.print("  Rep "); Serial.print(j + 1);
            Serial.print(": Duration = "); Serial.print(sets[i].durations[j]);
            Serial.print("s, ROM = "); Serial.print(sets[i].roms[j]);
            Serial.print(" inches, Tempo = ");
            Serial.print(sets[i].tempos[j][0]); Serial.print("-");
            Serial.print(sets[i].tempos[j][1]); Serial.print("-");
            Serial.println(sets[i].tempos[j][2]);
        }
    }
    Serial.println("========================");
}
