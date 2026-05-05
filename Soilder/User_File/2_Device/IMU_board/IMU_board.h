#pragma once

#include "BMI088.h"
#include "IST8310.h"
#include <math.h>

class IMU_Board {
public:
    void Init();
    void Update(float dt);

    float roll, pitch, yaw;

private:
    BMI088 imu;
    IST8310 mag;

    // gyro bias
    float bgx = 0, bgy = 0, bgz = 0;

    // 内部状态（弧度）
    float r = 0, p = 0, y = 0;

    void GyroBiasUpdate(float gx, float gy, float gz, float ax, float ay, float az);
};