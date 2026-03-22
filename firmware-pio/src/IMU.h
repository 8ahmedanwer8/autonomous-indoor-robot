#pragma once
#include <Arduino.h>

bool setupImu();
void updateImu();

bool isImuReady();

float getImuHeadingRad();
float getImuHeadingDeg();

float getImuMagX_uT();
float getImuMagY_uT();
float getImuMagZ_uT();

float getImuGyroX_radps();
float getImuGyroY_radps();
float getImuGyroZ_radps();

float getImuAccelX_mps2();
float getImuAccelY_mps2();
float getImuAccelZ_mps2();