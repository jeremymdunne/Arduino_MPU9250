#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>

#define RADIANS_TO_DEGREES 57.2957


#define MPU9250_ADDR_0 0x68
#define MPU9250_ADDR_1 0x69
#define MPU9250_SELF_TEST_X 0x0D
#define MPU9250_SELF_TEST_Y 0x0E
#define MPU9250_SELF_TEST_Z 0x0F
#define MPU9250_SELF_TEST_A 0x10
#define MPU9250_SMPLRT_DIV  0x19
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D
#define MPU9250_LP_ACCEL_ODR 0x1E
#define MPU9250_WOM_THR 0x1F
#define MPU9250_FIFO_ENABLE 0x23
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_SLV0_ADDR 0x25
#define MPU9250_I2C_SLV0_REG 0x26
#define MPU9250_I2C_SLV0_CTRL 0x27
#define MPU9250_I2C_MST_STATUS 0x36
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET 0x68
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_FIFO_COUNTH 0x72
#define MPU9250_FIFO_COUNTL 0x73
#define MPU9250_FIFO_R_W 0x74
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_WHO_AM_I_RESPONSE 0x71

#define AK8963_ADDR 0x0C
#define AK8963_WHO_AM_I  0x00
#define AK8963_WHO_AM_I_RESPONSE  0x48
#define AK8963_INFO 0x01
#define AK8963_STATUS_1 0x02
#define AK8963_MAG_X_L 0x03
#define AK8963_MAG_X_H 0x04
#define AK8963_MAG_Y_L 0x05
#define AK8963_MAG_Y_H 0x06
#define AK8963_MAG_Z_L 0x07
#define AK8963_MAG_Z_H 0x08
#define AK8963_STATUS_2 0x09
#define AK8963_CONTROL_1 0x0A
#define AK8963_CONTROL_2 0x0B
#define AK8963_SELF_TEST 0x0C
#define AK8963_MAG_ADJUSTMENT_X 0x10
#define AK8963_MAG_ADJUSTMENT_Y 0x11
#define AK8963_MAG_ADJUSTMENT_Z 0x12


#define COMPLEMENTARY_FILTER_KP .90


enum MPU9250_gyro_range{
  MPU9250_GYRO_RANGE_250_DPS = 0x00,
  MPU9250_GYRO_RANGE_500_DPS = 0x01,
  MPU9250_GYRO_RANGE_1000_DPS = 0x02,
  MPU9250_GYRO_RANGE_2000_DPS = 0x03
};

enum MPU9250_accel_range{
  MPU9250_ACCEL_RANGE_2_GPS = 0x00,
  MPU9250_ACCEL_RANGE_4_GPS = 0x01,
  MPU9250_ACCEL_RANGE_8_GPS = 0x02,
  MPU9250_ACCEL_RANGE_16_GPS = 0x03
};

enum MPU9250_clock_source{
  MPU9250_CLOCK_SOURCE_8_MHZ = 0x00,
  MPU9250_CLOCK_SOURCE_X_GYRO = 0x01,
  MPU9250_CLOCK_SOURCE_Y_GYRO = 0x02,
  MPU9250_CLOCK_SOURCE_Z_GYRO = 0x03
};

enum AK8963_mag_range{
  AK8963_14_BIT = 0x00,
  AK8963_16_BIT = 0x01
};

enum AK8963_mag_mode{
  AK8963_POWER_DOWN = 0x00,
  AK8963_SINGLE_MEASUREMENT = 0x01,
  AK8963_CONTINUOUS_MODE_1 = 0x02,
  AK8963_CONTINUOUS_MODE_2 = 0x06,
  AK8963_FUSE_ROM_MODE = 0x0F
};

struct Vector3f{
  float x = 0;
  float y = 0;
  float z = 0;
};

struct Vector3i{
  int x = 0;
  int y = 0;
  int z = 0;
};

struct MPU9250_Raw_Data{
  Vector3i gyro, accel, mag;
  int temp;
};

struct MPU9250_Scaled_Data{
  Vector3f gyro,accel, mag;
  float temp;
};

struct MPU9250_Data{
  Vector3f gyro, accel, mag, orientation;
  float rawHeading;
  float temp;
};


enum MPU9250_orientation{ //these reference the pull of gravity
  MPU9250_ORIENTATION_Z_DOWN,
  MPU9250_ORIENTATION_Z_UP,
  MPU9250_ORIENTATION_Y_DOWN,
  MPU9250_ORIENTATION_Y_UP,
  MPU9250_ORIENTATION_X_DOWN,
  MPU9250_ORIENTATION_x_UP,
};



class MPU9250{
public:
  int begin();
  int begin(MPU9250_gyro_range gyroRange, MPU9250_accel_range accelRange);
  void resetDevice();
  void setClockSource(MPU9250_clock_source source);
  void setAccelRange(MPU9250_accel_range range);
  void setGyroRange(MPU9250_gyro_range range);
  void getTempData(int *temp);
  void getAccelData(Vector3f *vec);
  void getGyroData(Vector3f *vec);
  void getScaledData(MPU9250_Scaled_Data *data);
  void getData(MPU9250_Data *data);
  void zero();
  void getRawData(MPU9250_Raw_Data * data);
  void calibrateGyroOffsets();
  void calibrateAccel();
  void setGyroCalibrationOffsets(float xOffset, float yOffset, float zOffset);
  void setAccelCalibrationOffsets(float xOffset, float yOffset, float zOffset);
  void setAccelCalibrationScales(float xScale, float yScale, float zScale);
  void setMagnetometerCalibrationOffsets(float xOffset, float yOffset, float zOffset);
  void setMagnetometerCalibrationScales(float xScale, float yScale, float zScale);
  void getGyroCalibrationOffsets(float *xOffset, float *yOffset, float *zOffset);
  void getAccelCalibrationOffsets(float *xOffset, float *yOffset, float *zOffset);
  void getAccelCalibrationScales(float *xScale, float *yScale, float *zScale);
  void getMagnetometerCalibrationOffsets(float *xOffset, float *yOffset, float *zOffset);
  void getMagnetometerCalibrationScales(float *xScale, float *yScale, float *zScale);

private:
  MPU9250_orientation mpuOrientation = MPU9250_ORIENTATION_Z_DOWN;

  float magX_Offset = -35;
  float magY_Offset = -210;
  float magZ_Offset = -155;
  float magX_Scale = 1.09, magY_Scale = 1.0, magZ_Scale = 1.0;
  long timeAtLastRead = 0;
  float magnetometerCalibrationOffsets[3] = {0,0,0}; //xyz in mTesla
  float magnetometerCalibrationScales[3] = {1,1,1}; //xyz in mTesla
  float accelCalibrationOffsets[3] = {0,0,0}; //xyz in g
  float accelCalibrationScales[3] = {1,1,1}; //xyz in g
  float gyroCalibrationOffsets[3] = {0,0,0}; //xyz in deg/sec
  float orientationOffsets[3] = {0,0,0}; //from the "zero" point
  MPU9250_Raw_Data rawData;
  MPU9250_Raw_Data preRotated;
  MPU9250_Scaled_Data scaledData;
  MPU9250_Data runningData;
  float xAcc, yAcc;
  long tempTime;
  int mpuAddr = MPU9250_ADDR_0;
  float gyroScale = calculateGyroScale(MPU9250_GYRO_RANGE_250_DPS);
  float accelScale = calculateAccelScale(MPU9250_ACCEL_RANGE_2_GPS);
  float magScale = calculateMagScale(AK8963_16_BIT);
  int akValueL, akValueH;
  int i2cReadValue;
  void update();
  void tiltCompensateMagnetometer(MPU9250_Scaled_Data *scaleData);
  void enableBypass();
  void getAllData(MPU9250_Raw_Data *rawData);
  void applyFilter(MPU9250_Scaled_Data *scaleData);
  void scaleData(MPU9250_Raw_Data *raw, MPU9250_Scaled_Data *scaledData);
  void normalizeGyro(MPU9250_Scaled_Data *data, long deltaMicros);
  float calculateMagScale(int scale);
  int initAK8963();
  float scaleTemp(int temp);
  bool checkMPU();
  float calculateGyroScale(int scale);
  float calculateAccelScale(int scale);
  void mpuWrite8(int reg, int value);
  void mpuWrite8(int reg);
  int mpuRead8(int reg);
  int mpuRead16(int regLow);
  int ak8963Read8(int reg);
  void ak8963Write8(int reg, int value);
  void ak8963Write8(int value);
  bool checkAK8963();
  int ak8963Read16(int reg);
};

#endif
