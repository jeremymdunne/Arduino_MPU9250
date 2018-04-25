#include <Arduino.h>
#include "MPU9250.h"

MPU9250 imu;
void setup() {
    // put your setup code here, to run once:
    delay(5000);
    Serial.begin(115200);
    Serial.println("Hello World!");
    //while(!Serial.available());
    if(imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS) < 0){
      Serial.println("IMU init Fail!");
      while(true);
    }

    Serial.println("IMU init Success!");
    Serial.println("Calibrating Gyroscope!");
    imu.calibrateGyroOffsets();
    float xOffset, yOffset, zOffset;
    imu.getGyroCalibrationOffsets(&xOffset, &yOffset, &zOffset);
    Serial.println("Gyro Offsets: X:" + String(xOffset) + " Y:" + String(yOffset) + " Z:" + String(zOffset));
    imu.zero();
    imu.setMagnetometerCalibrationOffsets(-36.35, 40.36, -140.07);
    imu.setMagnetometerCalibrationScales(1.0, 1.03, .92);
    imu.setDataFuseMode(MPU9250_DATA_FUSE_GYRO_MAG_AUTO_ACCEL);
    //imu.calibrateMagnetometer();

}
//MPU9250_Raw_Data imuData;
MPU9250_Data imuData;
long timeStart = 0;
void loop() {
    timeStart = micros();
    imu.getData(&imuData);
    //Serial.println("Ax:" + String(imuData.accel.x) + " Ay:"+ String(imuData.accel.y) + " Az:"+ String(imuData.accel.z) + " Gx:" + String(imuData.gyro.x) + " Gy:" + String(imuData.gyro.y) + " Gz:" + String(imuData.gyro.z) + " Temp:" + String(imuData.temp));
    //Serial.println(String(micros() - timeStart));
    //Serial.println("Mag X:" + String(imuData.mag.x) + " Y:" + String(imuData.mag.y) + " Z:" + String(imuData.mag.z));
    //Serial.println("Orientation: X:" + String(imuData.orientation.x) + " Y:" + String(imuData.orientation.y) + " Z:" + String(imuData.orientation.z) + " Raw Heading:" + String(imuData.rawHeading));
    //Serial.println("Gyro Z: " + String(imuData.gyro.z));
    //Serial.println("Heading: " + String(imuData.rawHeading) + "Mx:" + String(imuData.mag.x) + " My:" + String(imuData.mag.y) + " Mz:" + String(imuData.mag.z));
    Serial.println("@{X:" + String(imuData.orientation.x) + ";Y:" + String(imuData.orientation.y) + ";Z:" + String(imuData.orientation.z) + "}@");
    Serial.println("Linear Acceleration: " + String(imuData.linearAcceleration));
    Serial.print("DATA FUSE MODE: ");
    switch(imuData.dataFuseMode){
      case MPU9250_DATA_FUSE_FULL_9_DOF:
        Serial.println("9_DOF");
        break;
      case MPU9250_DATA_FUSE_GYRO_MAG_AUTO_ACCEL:
        Serial.println("GYRO MAG AUTO ACCEL");
        break;
      case MPU9250_DATA_FUSE_GYRO_MAG_AUTO_NO_ACCEL:
        Serial.println("GYRO MAG AUTO NO ACCEL");
        break;
    }
    delay(50);
    // put your main code here, to run repeatedly:
}
