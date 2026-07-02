#include "IMU.h"
#include "IMU_ICM20948.h"
#include "IMU_MPU6500.h"
#include "IMU_backend_config.h"

bool setupImu()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return setupImuIcm20948();
#else
  return setupImuMpu6500();
#endif
}

void updateImu()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  updateImuIcm20948();
#else
  updateImuMpu6500();
#endif
}

bool isImuReady()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return isImuIcm20948Ready();
#else
  return isImuMpu6500Ready();
#endif
}

float getImuHeadingRad()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948HeadingRad();
#else
  return getImuMpu6500HeadingRad();
#endif
}

float getImuHeadingDeg()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948HeadingDeg();
#else
  return getImuMpu6500HeadingDeg();
#endif
}

float getImuMagX_uT()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948MagX_uT();
#else
  return getImuMpu6500MagX_uT();
#endif
}

float getImuMagY_uT()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948MagY_uT();
#else
  return getImuMpu6500MagY_uT();
#endif
}

float getImuMagZ_uT()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948MagZ_uT();
#else
  return getImuMpu6500MagZ_uT();
#endif
}

float getImuGyroX_radps()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948GyroX_radps();
#else
  return getImuMpu6500GyroX_radps();
#endif
}

float getImuGyroY_radps()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948GyroY_radps();
#else
  return getImuMpu6500GyroY_radps();
#endif
}

float getImuGyroZ_radps()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948GyroZ_radps();
#else
  return getImuMpu6500GyroZ_radps();
#endif
}

float getImuAccelX_mps2()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948AccelX_mps2();
#else
  return getImuMpu6500AccelX_mps2();
#endif
}

float getImuAccelY_mps2()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948AccelY_mps2();
#else
  return getImuMpu6500AccelY_mps2();
#endif
}

float getImuAccelZ_mps2()
{
#if IMU_BACKEND == IMU_BACKEND_ICM20948
  return getImuIcm20948AccelZ_mps2();
#else
  return getImuMpu6500AccelZ_mps2();
#endif
}
