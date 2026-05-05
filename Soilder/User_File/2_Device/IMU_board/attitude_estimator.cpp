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
        // 4. 磁力计校准记录及数据处理
        // ---------------------------------------------------------
        // 传入原始 mx, my 进行动态阵列记录并适时生成校准参数
        CalibrateMag(mx, my);

        // 应用硬磁和软磁修正
        float mx_cal = (mx - mag_offset_x) * mag_scale_x;
        float my_cal = (my - mag_offset_y) * mag_scale_y;
        float mz_cal = mz; // 二维罗盘校准暂不处理Z轴
        
        // ---------------------------------------------------------
        // 5. 磁力计倾角补偿 (Tilt Compensation)
        // 使用校准后的磁力计数据
        // ---------------------------------------------------------
        float cos_r = cosf(r);
        float sin_r = sinf(r);
        float cos_p = cosf(p);
        float sin_p = sinf(p);
        
        float mx_comp = mx_cal * cos_p + mz_cal * sin_p;
        float my_comp = mx_cal * sin_r * sin_p + my_cal * cos_r - mz_cal * sin_r * cos_p;
        
        // 得到补偿后的磁力计绝对偏航角 (范围 -PI 到 PI)
        float mag_yaw = atan2f(my_comp, mx_comp);

        // ---------------------------------------------------------
        // 6. 修复 Yaw 过零点乱跳问题 (最短路径融合)
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

        // 7. 最终转回度 (deg) 给飞控
        Roll  = r * 57.29577951f;
        Pitch = p * 57.29577951f;
        Yaw   = y * 57.29577951f;
		printf("{pos}%.5f\n", Yaw);
    }
}