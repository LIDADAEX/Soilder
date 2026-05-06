#include "attitude_estimator.h"
#include <stdio.h>

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

    // 初始化磁力计校准参数
    mag_offset_x = mag_offset_y = 0.0f;
    mag_scale_x = mag_scale_y = 1.0f;
    valid_bin_count = 0;
    for (int i = 0; i < 180; i++) {
        Mag_Bins[i].valid = false;
    }
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 内部磁力计校准记录函数
void Class_Attitude_Estimator::CalibrateMag(float mx, float my) {
    // 1. 获取当前原始磁场朝向 (-180 到 180)
    float raw_angle = atan2f(my, mx) * 57.29577951f; 
    
    // 2. 映射到 0~179 的数组索引 (每2度一个)
    int bin = (int)(raw_angle + 180.0f) / 2;
    if (bin < 0) bin = 0;
    if (bin > 179) bin = 179;
    
    // 3. 记录数据
    if (!Mag_Bins[bin].valid) {
        Mag_Bins[bin].valid = true;
        valid_bin_count++;
    }
    Mag_Bins[bin].mx = mx;
    Mag_Bins[bin].my = my;
    
    // 4. 计算对向的角度索引 (相差180度 = 相差90个bin)
    int opposite_bin = (bin + 90) % 180;
    
    // 5. 触发条件：当前和对向都存在数据，且总收集数据量达到45个（至少转过了90度的连续区间）
    if (Mag_Bins[opposite_bin].valid && valid_bin_count >= 45) {
        float min_x = 1e9f, max_x = -1e9f;
        float min_y = 1e9f, max_y = -1e9f;
        
        // 寻找最大最小值
        for (int i = 0; i < 180; i++) {
            if (Mag_Bins[i].valid) {
                if (Mag_Bins[i].mx < min_x) min_x = Mag_Bins[i].mx;
                if (Mag_Bins[i].mx > max_x) max_x = Mag_Bins[i].mx;
                if (Mag_Bins[i].my < min_y) min_y = Mag_Bins[i].my;
                if (Mag_Bins[i].my > max_y) max_y = Mag_Bins[i].my;
            }
        }
        
        // 软磁与硬磁校准计算
        // 硬磁偏置 (Hard-Iron)
        mag_offset_x = (max_x + min_x) / 2.0f;
        mag_offset_y = (max_y + min_y) / 2.0f;
        
        // 软磁缩放比例 (Soft-Iron)
        float radius_x = (max_x - min_x) / 2.0f;
        float radius_y = (max_y - min_y) / 2.0f;
        
        if (radius_x > 0.001f && radius_y > 0.001f) {
            float avg_radius = (radius_x + radius_y) / 2.0f;
            mag_scale_x = avg_radius / radius_x;
            mag_scale_y = avg_radius / radius_y;
        }
        
        // 清空数组重新记录，等待下一次半圆运动
        for (int i = 0; i < 180; i++) {
            Mag_Bins[i].valid = false;
        }
        valid_bin_count = 0;
    }
}

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
        float gx_rad = gx * 0.01745329251f;
        float gy_rad = gy * 0.01745329251f;
        float gz_rad = gz * 0.01745329251f;

        // 1. 更新 Roll 和 Pitch (互补滤波)
        float accel_roll = atan2f(ay_f, az_f);
        float accel_pitch = -asinf(ax_f / norm);
        r = 0.995f * (r + gx_rad * dt) + 0.005f * accel_roll;
        p = 0.995f * (p + gy_rad * dt) + 0.005f * accel_pitch;

        // 2. 磁力计倾角补偿 (依然需要，否则姿态变了，角度一致性就丢了)
        // 注意：这里建议先用你的 CalibrateMag 处理一下，哪怕不准也能提高一致性
        CalibrateMag(mx, my);
        float mx_cal = (mx - mag_offset_x) * mag_scale_x;
        float my_cal = (my - mag_offset_y) * mag_scale_y;

        float cos_r = cosf(r), sin_r = sinf(r);
        float cos_p = cosf(p), sin_p = sinf(p);
        
        float mx_comp = mx_cal * cos_p + mz * sin_p;
        float my_comp = mx_cal * sin_r * sin_p + my_cal * cos_r - mz * sin_r * cos_p;
        
        // 得到当前环境下的“磁场特征角度”
        current_mag_yaw = atan2f(my_comp, mx_comp);

        // 3. Yaw 轴处理逻辑
        y += gz_rad * dt; // 纯靠陀螺仪积分

        if (!has_set_ref) {
            // 初始状态：记录当前磁场角度作为“零位地标”
            target_mag_yaw = current_mag_yaw;
            y = 0.0f; 
            has_set_ref = true;
        } else {
            // 计算当前磁场角度与参考地标的偏差
            float mag_err = current_mag_yaw - target_mag_yaw;
            
            // 归一化到 [-PI, PI]
            while (mag_err >  M_PI) mag_err -= 2.0f * M_PI;
            while (mag_err < -M_PI) mag_err += 2.0f * M_PI;

            // 4. 特征点对齐：如果磁场回到了地标附近 (比如误差在 2° 以内)
            if (fabsf(mag_err) < (2.0f * 0.01745329f)) {
                // 不要直接暴力赋值 y = 0，那样控制会抖动
                // 采用极缓慢的融合，或者当满足特定条件时微调
                float yaw_to_zero_err = 0.0f - y; 
                while (yaw_to_zero_err >  M_PI) yaw_to_zero_err -= 2.0f * M_PI;
                while (yaw_to_zero_err < -M_PI) yaw_to_zero_err += 2.0f * M_PI;

                // 只有当地标对齐时，才利用这 5% 的权重修正陀螺仪漂移
                y += 0.05f * yaw_to_zero_err; 
            }
        }

        // 保证 y 在 [-PI, PI]
        while (y >  M_PI) y -= 2.0f * M_PI;
        while (y < -M_PI) y += 2.0f * M_PI;

        // 最终输出
        Roll  = r * 57.29577951f;
        Pitch = p * 57.29577951f;
        Yaw   = y * 57.29577951f;
    }
}