#pragma once

bool setupImuIcm20948();
void updateImuIcm20948();

bool isImuIcm20948Ready();

float getImuIcm20948HeadingRad();
float getImuIcm20948HeadingDeg();

float getImuIcm20948MagX_uT();
float getImuIcm20948MagY_uT();
float getImuIcm20948MagZ_uT();

float getImuIcm20948GyroX_radps();
float getImuIcm20948GyroY_radps();
float getImuIcm20948GyroZ_radps();

float getImuIcm20948AccelX_mps2();
float getImuIcm20948AccelY_mps2();
float getImuIcm20948AccelZ_mps2();
