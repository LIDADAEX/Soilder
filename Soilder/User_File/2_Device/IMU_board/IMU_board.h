#pragma once

#include "BMI088.h"
#include "IST8310.h"
#include "attitude_estimator.h"

class IMU_Board {
public:
    void Init(); // 负责调用 imu.Init(), mag.Init(), estimator.Init()
    void Update(float dt);

    float roll, pitch, yaw;
    BMI088 imu;
    IST8310 mag;

private:
    Class_Attitude_Estimator estimator;

    // 零偏相关
    float bgx = 0, bgy = 0, bgz = 0;
    uint32_t bcnt = 0;
    void GyroBiasUpdate(float gx, float gy, float gz, float ax, float ay, float az);
};