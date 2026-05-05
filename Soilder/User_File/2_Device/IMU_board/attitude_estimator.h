#pragma once

#include "1_Middleware/2_Algorithm/Filter/alg_filter.h"
#include "arm_math.h"

static const float32_t FIR_Coeffs_30Hz[32] = {
    -0.001550f, -0.002824f, -0.003444f, -0.002390f, 
     0.001089f,  0.008035f,  0.018241f,  0.030805f, 
     0.044155f,  0.056382f,  0.065582f,  0.070146f, 
     0.069151f,  0.062402f,  0.050450f,  0.034503f, 
     0.034503f,  0.050450f,  0.062402f,  0.069151f, 
     0.070146f,  0.065582f,  0.056382f,  0.044155f, 
     0.030805f,  0.018241f,  0.008035f,  0.001089f, 
    -0.002390f, -0.003444f, -0.002824f, -0.001550f
};

class Class_Attitude_Estimator {
   public:
    float Roll, Pitch, Yaw;

    // 使用 DSP 库的 FIR 实例
    arm_fir_instance_f32 S_X, S_Y, S_Z;
    float32_t State_X[32 + 1], State_Y[32 + 1], State_Z[32 + 1];  // 状态缓冲区
    float32_t Coeffs[32];                                         // 滤波器系数

    void Init() {
        for(int i = 0; i < 32; i++) {
            Coeffs[i] = FIR_Coeffs_30Hz[i];
        }

        // 1. 如果使用 ARM DSP FIR，必须初始化这个结构体
        // 这里的 Coeffs 数组需要提前填入你用 Fourier 计算出来的系数
        arm_fir_init_f32(&S_X, 32, Coeffs, State_X, 1);
        arm_fir_init_f32(&S_Y, 32, Coeffs, State_Y, 1);
        arm_fir_init_f32(&S_Z, 32, Coeffs, State_Z, 1);

        Roll = Pitch = Yaw = 0.0f;
    }

    void Update(float ax, float ay, float az, float gx, float gy, float gz, float dt);
};