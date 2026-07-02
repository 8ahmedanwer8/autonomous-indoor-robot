#pragma once

bool setupImuMpu6500();
void updateImuMpu6500();

bool isImuMpu6500Ready();

float getImuMpu6500HeadingRad();
float getImuMpu6500HeadingDeg();

float getImuMpu6500MagX_uT();
float getImuMpu6500MagY_uT();
float getImuMpu6500MagZ_uT();

float getImuMpu6500GyroX_radps();
float getImuMpu6500GyroY_radps();
float getImuMpu6500GyroZ_radps();

float getImuMpu6500AccelX_mps2();
float getImuMpu6500AccelY_mps2();
float getImuMpu6500AccelZ_mps2();
