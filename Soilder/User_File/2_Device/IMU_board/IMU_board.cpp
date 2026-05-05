#include "IMU_board.h"

void IMU_Board::Update(float dt)
{
    auto data = imu.GetData();
    auto m = mag.GetData();

    // === 单位统一 ===
    float gx = data.gx * 0.0174533f;
    float gy = data.gy * 0.0174533f;
    float gz = data.gz * 0.0174533f;

    float ax = data.ax;
    float ay = data.ay;
    float az = data.az;

    float mx = m.mx;
    float my = m.my;
    float mz = m.mz;

    // === 动态零偏 ===
    GyroBiasUpdate(gx, gy, gz, ax, ay, az);

    gx -= bgx;
    gy -= bgy;
    gz -= bgz;

    // === 加速度解算姿态 ===
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.5f) return;

    float accel_roll  = atan2f(ay, az);
    float accel_pitch = -asinf(ax / norm);

    // === 磁力计 yaw ===
    float mag_yaw = atan2f(my, mx);

    // === 互补滤波 ===
    r = 0.98f * (r + gx * dt) + 0.02f * accel_roll;
    p = 0.98f * (p + gy * dt) + 0.02f * accel_pitch;

    // yaw 用磁力计修正
    y = 0.98f * (y + gz * dt) + 0.02f * mag_yaw;

    // 输出（角度）
    roll  = r * 57.29578f;
    pitch = p * 57.29578f;
    yaw   = y * 57.29578f;
}