/**
 * @file crt_chassis.cpp
 * @brief 底盘控制与航位推算系统实现
 * @details 
 * 1. 采用冗余观测器逻辑：通过 X、Y 两组电机互相校验，自动补偿打滑。
 * 2. 支持世界坐标系控制：内置旋转矩阵与一阶相位延迟预测补偿。
 * 3. 具备电机在线状态监测与安全保护（掉线重置原点）。
 */

#include "crt_chassis.h"
#include <cmath>

/* ========================================================================= */
/* =                         1. WorldPosition 航位推算                      = */
/* ========================================================================= */

void WorldPosition::init(const int32_t *x_p, const int32_t *x_m,
                         const int32_t *y_p, const int32_t *y_m,
                         Config cfg) {
    m_x_p_ptr = x_p; m_x_m_ptr = x_m;
    m_y_p_ptr = y_p; m_y_m_ptr = y_m;
    m_cfg = cfg;

    // 物理系数计算: 距离 = (脉冲 / 单圈总脉冲) * (2 * PI * 半径)
    m_tick_to_dist = (2.0f * PI * m_cfg.wheel_radius) / (m_cfg.ppr * m_cfg.ratio);

    m_x_correction_m = 0.0f;
    m_y_correction_m = 0.0f;
    m_now_angle = 0.0f;

    // 记录初始时刻的平均读数作为参考零点
    float init_x_rot = (*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist;
    float init_y_rot = (*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist;
    m_last_shared_rot_m = (init_x_rot + init_y_rot) / 2.0f;
}

/** @brief 重置参考原点：用于电机恢复在线时同步读数，防止角度突跳 */
void WorldPosition::reset_origin() {
    float cur_x = (*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist + m_x_correction_m;
    float cur_y = (*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist + m_y_correction_m;
    m_last_shared_rot_m = (cur_x + cur_y) / 2.0f;
}

void WorldPosition::update() {
    // 1. 获取当前读数（包含历史补偿量）
    float current_x_rot_m = (*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist + m_x_correction_m;
    float current_y_rot_m = (*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist + m_y_correction_m;

    // 2. 计算本帧增量
    float dx = current_x_rot_m - m_last_shared_rot_m;
    float dy = current_y_rot_m - m_last_shared_rot_m;
    float chosen_d_rot = 0.0f;

    // 3. 冗余校验逻辑：
    // 原理：正常旋转时 dx 应等于 dy。若不等，说明有一组电机打滑空转。
    // 逻辑：取绝对值较小（位移较少）的一组作为真实值，并修正另一组的偏差。
    if (std::abs(dx - dy) > 0.0001f) {
        if (std::abs(dx) <= std::abs(dy)) {
            chosen_d_rot = dx;
            m_y_correction_m += (dx - dy); // 强制 Y 组对齐 X 组进度
            LOG_WARNING("Y轴疑似打滑，已向X轴对齐");
        } else {
            chosen_d_rot = dy;
            m_x_correction_m += (dy - dx); // 强制 X 组对齐 Y 组进度
            LOG_WARNING("X轴疑似打滑，已向Y轴对齐");
        }
    } else {
        chosen_d_rot = (dx + dy) / 2.0f; // 无明显误差则取平均
    }

    // 4. 增量转角度：theta_inc = arc / (2 * L)
    // 这里的 2.0f 是因为我们用了两组对角电机的位移和
    float d_theta = chosen_d_rot / (2.0f * m_cfg.wheelbase);
    m_now_angle += d_theta;

    // 5. 角度归一化 [-PI, PI]
    if (m_now_angle > PI)  m_now_angle -= 2.0f * PI;
    if (m_now_angle < -PI) m_now_angle += 2.0f * PI;

    // 6. 更新历史读数
    m_last_shared_rot_m += chosen_d_rot;
}

/* ========================================================================= */
/* =                            2. Chassis 底盘核心                         = */
/* ========================================================================= */

void Chassis::chassis_init(Class_Motor_DJI_C620 &x_p, Class_Motor_DJI_C620 &x_m, 
                           Class_Motor_DJI_C620 &y_p, Class_Motor_DJI_C620 &y_m, 
                           WorldPosition::Config* cfg_in) {
    m_motors[0] = &x_p; m_motors[1] = &x_m;
    m_motors[2] = &y_p; m_motors[3] = &y_m;

    WorldPosition::Config cfg;
    if (cfg_in == nullptr) {
        cfg.ppr = 8192.0f; cfg.ratio = 19.2032f; // 3591/187
        cfg.wheel_radius = 0.076f; cfg.wheelbase = 0.25f;
    } else {
        cfg = *cfg_in;
    }

    // 初始化推算器
    m_worldPosition.init(
        &x_p.Get_Rx_Data().Total_Encoder, &x_m.Get_Rx_Data().Total_Encoder,
        &y_p.Get_Rx_Data().Total_Encoder, &y_m.Get_Rx_Data().Total_Encoder,
        cfg);

    m_radius = cfg.wheel_radius;
    m_gear_ratio = cfg.ratio;
    // 物理量转换系数：线速度 V -> 电机端角速度 Omega
    m_vel_to_omega_coeff = (1.0f / m_radius) * m_gear_ratio;
}

/**
 * @brief 底盘控制循环 (1ms)
 */
void Chassis::TIM_1ms_Calculate_PeriodElapsedCallback() {
    bool all_motors_healthy = true;

    // 1. 电机状态检查与 PID 计算
    for(uint8_t i = 0; i < 4; i++) {
        if(m_motors[i]->Get_Status() != Motor_DJI_Status_ENABLE) {
            all_motors_healthy = false;
            continue; 
        }
        m_motors[i]->TIM_Calculate_PeriodElapsedCallback();
        m_motors[i]->TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();
    }
    
    // 安全策略：电机不全则重置原点并停止
    if(!all_motors_healthy) {
        m_worldPosition.reset_origin(); 
        return;
    }

    // 无控制输入则直接跳过后续解算
    if(!m_target_vx && !m_target_vy && !m_target_vw) return;

    // 2. 更新位置信息
    m_worldPosition.update(); 

    float exec_vx = m_target_vx;
    float exec_vy = m_target_vy;

    // 3. 世界坐标系变换 (坐标映射与相位补偿)
    if (m_is_world_frame) {
        float angle_now = m_worldPosition.getAngle();
        
        // 计算真实旋转角速度 (用于一阶预测补偿角度延迟)
        float real_omega_w = (m_motors[0]->Get_Now_Omega() + m_motors[1]->Get_Now_Omega() + 
                              m_motors[2]->Get_Now_Omega() + m_motors[3]->Get_Now_Omega()) 
                              / (4.0f * m_gear_ratio); 
        
        // 预测角度 = 当前角度 + 速度 * 采样延迟补偿
        float angle_predict = angle_now + real_omega_w * (m_delay_comp_ms * 0.001f);

        // 旋转矩阵变换 (世界坐标 -> 机器人坐标)
        float sin_a = arm_sin_f32(angle_predict);
        float cos_a = arm_cos_f32(angle_predict);
        exec_vx = m_target_vx * cos_a + m_target_vy * sin_a;
        exec_vy = -m_target_vx * sin_a + m_target_vy * cos_a;
    }

    // 4. 逆运动学解算：计算四轮线速度
    // 注意：这里的方向符号取决于具体的物理建模排布
    float motor_v[4];
    motor_v[0] =  exec_vx + m_target_vw; // 前左
    motor_v[1] = -exec_vx + m_target_vw; // 后右
    motor_v[2] =  exec_vy + m_target_vw; // 前右
    motor_v[3] = -exec_vy + m_target_vw; // 后左

    // 5. 整体限幅保护：保持速度向量方向不变的前提下等比例缩小
    const float max_v = 3.5f; 
    float current_max = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (fabsf(motor_v[i]) > current_max) current_max = fabsf(motor_v[i]);
    }

    float scale = (current_max > max_v) ? (max_v / current_max) : 1.0f;

    // 6. 下发至电机
    for (int i = 0; i < 4; i++) {
        m_motors[i]->Set_Target_Omega(motor_v[i] * scale * m_vel_to_omega_coeff);
    }
}

/**
 * @brief 离线重连监测 (100ms)
 */
void Chassis::TIM_100ms_Alive_PeriodElapsedCallback(){
    static uint8_t cnt[4] = {0};

    for(uint8_t i = 0; i < 4; i++){
        if(cnt[i] >= 10) continue;

        m_motors[i]->TIM_100ms_Alive_PeriodElapsedCallback();
        
        if(m_motors[i]->Get_Status() != Motor_DJI_Status_ENABLE) {
            cnt[i]++;
            LOG_WARNING("电机" + std::to_string(i) + "掉线，重连中: " + std::to_string(cnt[i]));
            if(cnt[i] >= 10) LOG_ERROR("电机" + std::to_string(i) + "彻底丢失");
        } else {
            cnt[i] = 0; // 恢复在线，重置计数
        }
    }
}