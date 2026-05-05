#include "attitude_estimator.h"

#define DEG2RAD 0.01745329251f



void Class_Attitude_Estimator::Update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;
    
    float32_t ax_f, ay_f, az_f;

    // 2. 使用 DSP 库进行单点 FIR 滤波
    arm_fir_f32(&S_X, &ax, &ax_f, 1);
    arm_fir_f32(&S_Y, &ay, &ay_f, 1);
    arm_fir_f32(&S_Z, &az, &az_f, 1);

    // 3. 快速平方根计算向量模长
    float32_t sum_sq = ax_f * ax_f + ay_f * ay_f + az_f * az_f;
    float32_t norm;
    arm_sqrt_f32(sum_sq, &norm);

    if (norm > 0.1f) {
        // F407 的 FPU 处理 atan2f 和 asinf 非常快
        float accel_roll = atan2f(ay_f, az_f) * 57.29578f;
        float accel_pitch = -asinf(ax_f / norm) * 57.29578f;

        // 互补滤波融合
        Roll = 0.995f * (Roll + gx * dt) + 0.005f * accel_roll;
        Pitch = 0.995f * (Pitch + gy * dt) + 0.005f * accel_pitch;
        Yaw += gz * dt;
    }
}