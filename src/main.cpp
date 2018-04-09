#include <Arduino.h>
#include "MPU9250.h"

MPU9250 imu;
void setup() {
    // put your setup code here, to run once:
    delay(5000);
    Serial.begin(115200);
    Serial.println("Hello World!");
    //while(!Serial.available());
    if(imu.begin(MPU9250_GYRO_RANGE_250_DPS, MPU9250_ACCEL_RANGE_8_GPS) < 0){
      Serial.println("IMU init Fail!");
      while(true);
    }
    Serial.println("IMU init Success!");
    //delay(500);
    Serial.println("Calibrating Gyroscope!");
    imu.calibrateGyroOffsets();
    float xOffset, yOffset, zOffset;
    imu.getGyroCalibrationOffsets(&xOffset, &yOffset, &zOffset);
    Serial.println("Gyro Offsets: X:" + String(xOffset) + " Y:" + String(yOffset) + " Z:" + String(zOffset));
    //while(true);

    //imu.resetDevice();
    //Serial.println("Reset complete!");
}
//MPU9250_Raw_Data imuData;
MPU9250_Data imuData;
long timeStart = 0;
void loop() {
    timeStart = millis();
    imu.getData(&imuData);
    //imu.getAllData(&imuData);
    //Serial.println("Ax:" + String(imuData.accel.x) + " Ay:"+ String(imuData.accel.y) + " Az:"+ String(imuData.accel.z) + " Gx:" + String(imuData.gyro.x) + " Gy:" + String(imuData.gyro.y) + " Gz:" + String(imuData.gyro.z) + " Temp:" + String(imuData.temp));
    Serial.println("Orientation: X:" + String(imuData.orientation.x) + " Y:" + String(imuData.orientation.y) + " Z:" + String(imuData.orientation.z) + " Raw Heading:" + String(imuData.rawHeading));
    //Serial.println("Heading: " + String(imuData.rawHeading) + "Mx:" + String(imuData.mag.x) + " My:" + String(imuData.mag.y) + " Mz:" + String(imuData.mag.z));
    Serial.println(String(millis() - timeStart));
    delay(100);
    // put your main code here, to run repeatedly:
}
