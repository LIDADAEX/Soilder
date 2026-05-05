#pragma once
#include "arm_math.h"
#include <math.h>

class Class_Attitude_Estimator {
public:
    float Roll, Pitch, Yaw; // 最终输出 (角度 deg)

    void Init();
    // 增加 mx, my, mz，实现 9 轴融合
    void Update(float ax, float ay, float az, 
                float gx, float gy, float gz, 
                float mx, float my, float mz, float dt);

private:
    arm_fir_instance_f32 S_X, S_Y, S_Z;
    float32_t State_X[33], State_Y[33], State_Z[33]; // DSP规定：StateSize = numTaps + blockSize - 1
    float32_t Coeffs[32];
    
    // 内部状态保持 (弧度 rad)，避免单位错乱
    float r, p, y; 
};