#pragma once
#include "arm_math.h"
#include <math.h>

// 用于记录磁力计数据的结构体
struct MagRecord {
    float mx;
    float my;
    bool valid;
};

class Class_Attitude_Estimator {
public:
    float Roll, Pitch, Yaw; // 最终输出 (角度 deg)

    // 磁力计校准参数 (可以暴露出来用于保存到EEPROM/Flash)
    float mag_offset_x, mag_offset_y; // 硬磁偏移
    float mag_scale_x, mag_scale_y;   // 软磁缩放

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

    // --- 磁力计动态校准相关 ---
    MagRecord Mag_Bins[180]; // -180~180度，每2度一个记录点
    int valid_bin_count;     // 当前已记录的有效点数量

	float target_mag_yaw;    // 记录零位时的磁场特征角度
	bool  has_set_ref = false; // 是否已经捕获过参考点
	float current_mag_yaw;   // 当前实时计算出的磁场角度（用于观察
    
    void CalibrateMag(float mx, float my);
};