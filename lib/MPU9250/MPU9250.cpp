#include "MPU9250.h"


void MPU9250::getRawData(MPU9250_Raw_Data * data){
  getAllData(data);
}

void MPU9250::zero(){
  long startMillis = millis();
  for(int i = 0; i < 100; i ++){
    update();
    delay(5);
  }
  long endMillis = millis();
  dX_Offset = runningData.gyro.x / ((endMillis - startMillis)/1000.0);
  dY_Offset = runningData.gyro.y / ((endMillis - startMillis)/1000.0);
  dZ_Offset = runningData.gyro.z / ((endMillis - startMillis)/1000.0);
  //clear previous values

  //startMillis = millis();
  for(int i = 0; i < 50; i ++){
    update();
  }
  //endMillis = millis();
  xOffset = runningData.orientation.x;
  yOffset = runningData.orientation.y;
  //z can't be fixed
}


void MPU9250::getData(MPU9250_Data *data){
    if(timeAtLastRead == 0){
      update();
      delay(10);
    }
    update();
    *data = runningData;
    data->orientation.x -= xOffset;
    data->orientation.y -= yOffset;
    data->orientation.z -= zOffset;
}


void MPU9250::applyFilter(MPU9250_Scaled_Data *scaleData){
  xAcc = atan2f(scaleData->accel.y, scaleData->accel.z) *180.0/M_PI;
  yAcc = atan2f(scaleData->accel.x, scaleData->accel.z) * 180.0/M_PI;
  runningData.orientation.x = (runningData.orientation.x + scaleData->gyro.x)*COMPLEMENTARY_FILTER_KP + (1.0 - COMPLEMENTARY_FILTER_KP)*xAcc;
  runningData.orientation.y = (runningData.orientation.y + scaleData->gyro.y)*COMPLEMENTARY_FILTER_KP + (1.0 - COMPLEMENTARY_FILTER_KP)*yAcc;
  float heading = atan2(scaleData->mag.y , scaleData->mag.x) *180.0 / M_PI;
  if(heading < 0) heading += 360;
  else if(heading > 360) heading -= 360;
  runningData.rawHeading = heading;
  //Serial.println("gyro z:" + String(scaleData->gyro.z));
  if(runningData.orientation.z >= 270 && heading < 90.0){
    runningData.orientation.z = heading;
  }

  else if(runningData.orientation.z < 90 && heading > 270.0){
    runningData.orientation.z = heading;
  }
  runningData.orientation.z = (runningData.orientation.z + scaleData->gyro.z)*COMPLEMENTARY_FILTER_KP + (1.0 - COMPLEMENTARY_FILTER_KP)*heading;
}

void MPU9250::update(){
  tempTime = micros();
  getAllData(&rawData);
  scaleData(&rawData,&scaledData);
  normalizeGyro(&scaledData, tempTime - timeAtLastRead);
  runningData.accel = scaledData.accel;
  runningData.temp = scaledData.temp;
  runningData.gyro.x += scaledData.gyro.x;
  runningData.gyro.y += scaledData.gyro.y;
  runningData.gyro.z += scaledData.gyro.z;
  runningData.mag = scaledData.mag;
  applyFilter(&scaledData);
  timeAtLastRead = tempTime;
}

void MPU9250::normalizeGyro(MPU9250_Scaled_Data *data, long deltaMicros){
  data->gyro.x *= (float)(deltaMicros/1000000.0);
  data->gyro.y *= (float)(deltaMicros/1000000.0);
  data->gyro.z *= (float)(deltaMicros/1000000.0);
}


float MPU9250::scaleTemp(int temp){
  return temp/340.0 + 36.53;
}

void MPU9250::getScaledData(MPU9250_Scaled_Data *data){
  getAllData(&rawData);
  scaleData(&rawData, data);
}

void MPU9250::scaleData(MPU9250_Raw_Data *raw, MPU9250_Scaled_Data *data){
  data->gyro.x = raw->gyro.x / gyroScale - dX_Offset;
  data->gyro.y = -1.0*raw->gyro.y / gyroScale - dY_Offset;
  data->gyro.z = raw->gyro.z / gyroScale - dZ_Offset;
  data->accel.x = raw->accel.x / accelScale;
  data->accel.y = raw->accel.y / accelScale;
  data->accel.z = raw->accel.z / accelScale;
  data->temp = scaleTemp(raw->temp);
  data->mag.x = (raw->mag.x / magScale - magX_Offset) * magX_Scale;
  data->mag.y = (raw->mag.y / magScale - magY_Offset) * magY_Scale;
  data->mag.z = (raw->mag.z / magScale - magZ_Offset) * magZ_Scale;

}

float MPU9250::calculateAccelScale(int scale){
  switch (scale) {
    case MPU9250_ACCEL_RANGE_2_GPS:
      return 16384.0;
      break;
    case MPU9250_ACCEL_RANGE_4_GPS:
      return 8192.0;
      break;
    case MPU9250_ACCEL_RANGE_8_GPS:
      return 4096.0;
      break;
    case MPU9250_ACCEL_RANGE_16_GPS:
      return 2048.0;
      break;
  }
  return -1;
}

float MPU9250::calculateMagScale(int scale){
  switch (scale){
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case AK8963_14_BIT:
          return 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case AK8963_16_BIT:
          return 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
    }
}

float MPU9250::calculateGyroScale(int scale){
  switch (scale) {
    case MPU9250_GYRO_RANGE_250_DPS:
      return 131.0;
      break;
    case MPU9250_GYRO_RANGE_500_DPS:
      return 65.5;
      break;
    case MPU9250_GYRO_RANGE_1000_DPS:
      return 32.8;
      break;
    case MPU9250_GYRO_RANGE_2000_DPS:
      return 16.4;
      break;
  }
  return -1;
}


void MPU9250::getAllData(MPU9250_Raw_Data *raw){
  //Serial.println(mpuRead16(MPU9250_ACCEL_XOUT_H));
  mpuWrite8(MPU9250_ACCEL_XOUT_H);
  Wire.requestFrom(mpuAddr,14,true);
  raw->accel.x = Wire.read()<<8|Wire.read();
  raw->accel.y = Wire.read() <<8|Wire.read();
  raw->accel.z = Wire.read() <<8|Wire.read();
  raw->temp = Wire.read() <<8|Wire.read();
  raw->gyro.x = Wire.read() <<8|Wire.read();
  raw->gyro.y = Wire.read() <<8|Wire.read();
  raw->gyro.z = Wire.read() <<8|Wire.read();
  //check if new mag data avialble
  if(ak8963Read8(AK8963_STATUS_1) & 0x01){
    raw->mag.x = ak8963Read16(AK8963_MAG_X_L);
    raw->mag.y = ak8963Read16(AK8963_MAG_Y_L);
    raw->mag.z = ak8963Read16(AK8963_MAG_Z_L);
    ak8963Read8(AK8963_STATUS_2);
  }

}

int MPU9250::begin(){
  Wire.begin();
  if(!checkMPU()){
    return -1;
  }
  if(initAK8963() < 0){
    return -2;
  }
  resetDevice();
  setClockSource(MPU9250_CLOCK_SOURCE_X_GYRO);
  delay(100);
  setAccelRange(MPU9250_ACCEL_RANGE_2_GPS);
  setGyroRange(MPU9250_GYRO_RANGE_250_DPS);
  Serial.println(mpuRead8(MPU9250_CONFIG),BIN);

  return 0;
}

int MPU9250::begin(MPU9250_gyro_range gyroRange, MPU9250_accel_range accelRange){
  Wire.begin();
  if(!checkMPU()){
    return -1;
  }

  resetDevice();
  setClockSource(MPU9250_CLOCK_SOURCE_X_GYRO);
  delay(100);
  setAccelRange(accelRange);
  setGyroRange(gyroRange);
  enableBypass();
  if(initAK8963() < 0){
    return -2;
  }
  //Serial.println(mpuRead8(MPU9250_CONFIG),BIN);
  return 0;
}


void MPU9250::enableBypass(){
  int value = mpuRead8(MPU9250_INT_PIN_CFG);
  value &= 0b11111101;
  value |= 1 << 1;
  mpuWrite8(MPU9250_INT_PIN_CFG,value);
}

int MPU9250::initAK8963(){
  if(!checkAK8963()){
    return -1;
  }

  ak8963Write8(AK8963_CONTROL_1,0x00);
  delay(10);
  ak8963Write8(AK8963_CONTROL_1,AK8963_16_BIT << 4 | AK8963_CONTINUOUS_MODE_2);
  delay(10);
  return 0;
}

bool MPU9250::checkAK8963(){
  Serial.println("Mag init: Who am I: " + String(ak8963Read8(AK8963_WHO_AM_I)));
  if(ak8963Read8(AK8963_WHO_AM_I) != AK8963_WHO_AM_I_RESPONSE){
    return false;
  }
  return true;
}

void MPU9250::setClockSource(MPU9250_clock_source source){
  mpuWrite8(MPU9250_PWR_MGMT_1,source);
}

void MPU9250::setAccelRange(MPU9250_accel_range range){
    int value = mpuRead8(MPU9250_ACCEL_CONFIG);
    //mpuWrite8(MPU9250_ACCEL_CONFIG, value &=0b00011111);
    //mpuWrite8(MPU9250_ACCEL_CONFIG, value &=0b11100111);
    value &= 0b11100111;
    value |= range << 3;
    //Serial.println(value|(range  << 3),BIN);
    mpuWrite8(MPU9250_ACCEL_CONFIG, value);
    accelScale = calculateAccelScale(range);
}

void MPU9250::setGyroRange(MPU9250_gyro_range range){
  int value = mpuRead8(MPU9250_GYRO_CONFIG);
  value &= 0b11100111;
  value |= range << 3;
  mpuWrite8(MPU9250_GYRO_CONFIG,value);
  gyroScale = calculateGyroScale(range);
}



void MPU9250::resetDevice(){
  int value = mpuRead8(MPU9250_PWR_MGMT_1);
  //Serial.println(value, BIN);
  value &= 0b01111111;
  value |= 1 << 7;
  mpuWrite8(MPU9250_PWR_MGMT_1,value);
  delay(10);
  while(mpuRead8(MPU9250_PWR_MGMT_1) >> 7 != 0) delay(10);
  value = mpuRead8(MPU9250_PWR_MGMT_1);
  //Serial.println(value, BIN);
}


bool MPU9250::checkMPU(){
  if(mpuRead8(MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_RESPONSE){
    Serial.println(mpuRead8(MPU9250_WHO_AM_I),HEX);
    return false;
  }
  return true;
}

void MPU9250::mpuWrite8(int reg, int value){
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU9250::mpuWrite8(int reg){
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.endTransmission();
}

int MPU9250::mpuRead8(int reg){
  mpuWrite8(reg);
  Wire.requestFrom(mpuAddr,1);
  if(Wire.available() >= 1){
    return Wire.read();
  }
  return -1;
}

int MPU9250::mpuRead16(int regLow){
  mpuWrite8(regLow);
  Wire.requestFrom(mpuAddr,2);
  int value = -1;
  if(Wire.available() >= 2){
    value = Wire.read();
    value = value << 8;
    value |= Wire.read();
  }
  return value;
}

int MPU9250::ak8963Read8(int reg){
  ak8963Write8(reg);
  Wire.requestFrom(AK8963_ADDR,1);
  if(Wire.available() >= 1){
    return Wire.read();
  }
  return -1;
}

int MPU9250::ak8963Read16(int reg){
  int valueL,valueH;
  ak8963Write8(reg);
  Wire.requestFrom(AK8963_ADDR,2);
  if(Wire.available() >= 2){
    valueL = Wire.read();
    valueH = Wire.read();
    return valueH << 8 | valueL;
  }
  return -1;
}

void MPU9250::ak8963Write8(int reg, int value){
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU9250::ak8963Write8(int value){
  Wire.beginTransmission(AK8963_ADDR);
  //Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
