#include "attitude_estimator.h"

// 此处填入你的 32 阶系数
static const float32_t FIR_Coeffs_30Hz[32] = {
    -0.001550f, -0.002824f, -0.003444f, -0.002390f, 
     0.001089f,  0.008035f,  0.018241f,  0.030805f, 
     0.044155f,  0.056382f,  0.065582f,  0.070146f, 
     0.069151f,  0.062402f,  0.050450f,  0.034503f, 
     0.034503f,  0.050450f,  0.062402f,  0.069151f, 
     0.070146f,  0.065582f,  0.056382f,  0.044155f, 
     0.030805f,  0.018241f,  0.008035f,  0.001089f, 
    -0.002390f, -0.003444f, -0.002824f, -0.001550f}; 

void Class_Attitude_Estimator::Init() {
    for(int i = 0; i < 32; i++) {
        Coeffs[i] = FIR_Coeffs_30Hz[i];
    }
    arm_fir_init_f32(&S_X, 32, Coeffs, State_X, 1);
    arm_fir_init_f32(&S_Y, 32, Coeffs, State_Y, 1);
    arm_fir_init_f32(&S_Z, 32, Coeffs, State_Z, 1);

    Roll = Pitch = Yaw = 0.0f;
    r = p = y = 0.0f;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void Class_Attitude_Estimator::Update(float ax, float ay, float az, 
                                      float gx, float gy, float gz, 
                                      float mx, float my, float mz, float dt) {
    // 1. FIR 滤波加速度
    float32_t ax_f, ay_f, az_f;
    arm_fir_f32(&S_X, &ax, &ax_f, 1);
    arm_fir_f32(&S_Y, &ay, &ay_f, 1);
    arm_fir_f32(&S_Z, &az, &az_f, 1);

    float32_t sum_sq = ax_f * ax_f + ay_f * ay_f + az_f * az_f;
    float32_t norm;
    arm_sqrt_f32(sum_sq, &norm);

    if (norm > 0.1f) {
        // 2. 角速度转为 弧度/s
        float gx_rad = gx * 0.01745329251f;
        float gy_rad = gy * 0.01745329251f;
        float gz_rad = gz * 0.01745329251f;

        // 3. 先更新 Roll 和 Pitch (需要用它们来给磁力计做补偿)
        float accel_roll = atan2f(ay_f, az_f);
        float accel_pitch = -asinf(ax_f / norm);

        r = 0.995f * (r + gx_rad * dt) + 0.005f * accel_roll;
        p = 0.995f * (p + gy_rad * dt) + 0.005f * accel_pitch;

        // ---------------------------------------------------------
        // 4. 磁力计倾角补偿 (Tilt Compensation)
        // 将板体坐标系的磁力计数据投影到绝对水平面上
        // ---------------------------------------------------------
        float cos_r = cosf(r);
        float sin_r = sinf(r);
        float cos_p = cosf(p);
        float sin_p = sinf(p);
        
        float mx_comp = mx * cos_p + mz * sin_p;
        float my_comp = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p;
        
        // 得到补偿后的磁力计绝对偏航角 (范围 -PI 到 PI)
        float mag_yaw = atan2f(my_comp, mx_comp);

        // ---------------------------------------------------------
        // 5. 修复 Yaw 过零点乱跳问题 (最短路径融合)
        // ---------------------------------------------------------
        y += gz_rad * dt; // 先用陀螺仪积分出一个预测值

        // 计算磁力计和陀螺仪预测值之间的偏差
        float yaw_err = mag_yaw - y;
        
        // 将偏差强制约束在 -PI 到 PI 之间 (处理 +179度 和 -179度 之间的短路径)
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

        // 利用互补滤波消除累积误差 (权重给0.01或0.02)
        y += 0.01f * yaw_err; 

        // 保证 y 自身也在 -PI 到 PI 的合理范围内
        while (y >  M_PI) y -= 2.0f * M_PI;
        while (y < -M_PI) y += 2.0f * M_PI;

        // 6. 最终转回度 (deg) 给飞控
        Roll  = r * 57.29577951f;
        Pitch = p * 57.29577951f;
        Yaw   = y * 57.29577951f;
    }
}