#include "crt_chassis.h"

void WorldPosition::init(const int32_t *x_p, const int32_t *x_m,
                         const int32_t *y_p, const int32_t *y_m,
                         Config cfg)
{
    m_x_p_ptr = x_p;
    m_x_m_ptr = x_m;
    m_y_p_ptr = y_p;
    m_y_m_ptr = y_m;
    m_cfg = cfg;

    // 计算脉冲转物理位移的系数: 1 tick = 多少米
    // 距离 = (脉冲 / (线数 * 减速比)) * 周长
    m_tick_to_dist = (2.0f * PI * m_cfg.wheel_radius) / (m_cfg.ppr * m_cfg.ratio);

    // 初始化累计修正量为 0
    m_x_correction_m = 0.0f;
    m_y_correction_m = 0.0f;

    // 记录初始读数作为起始原点
    // 取当前 X 组和 Y 组旋转贡献的平均值作为“起跑线”
    float init_x_rot = (*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist;
    float init_y_rot = (*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist;
    m_last_shared_rot_m = (init_x_rot + init_y_rot) / 2.0f;

    m_now_angle = 0.0f;
}

void WorldPosition::update()
{
    // 1. 获取当前两组观测器的“原始位移总量”（单位：米）
    // 计算公式：(左轮+右轮) * 系数 + 历史修正量
    float current_x_rot_m = (*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist + m_x_correction_m;
    float current_y_rot_m = (*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist + m_y_correction_m;

    // 2. 计算这一帧相对于上一帧的增量
    float dx = current_x_rot_m - m_last_shared_rot_m;
    float dy = current_y_rot_m - m_last_shared_rot_m;

    float chosen_d_rot = 0.0f;

    // 3. 冗余校验逻辑：相信绝对值较小的一组（假设打滑表现为空转/数值偏大）
    if (std::abs(dx - dy) > 0.0001f)
    { // 极小死区防止浮点数抖动
        if (std::abs(dx) <= std::abs(dy))
        {
            // 相信 X 组
            chosen_d_rot = dx;
            // 【核心修正】：让 Y 组强行对齐 X 组的进度，补偿掉差值
            // 这样下一帧 dy = (next_y - current_y) + (dx - dy)，dy 就会等于 dx
            m_y_correction_m += (dx - dy);
            LOG_WARNING("Y轴疑似打滑，已向X轴对齐。");
        }
        else
        {
            // 相信 Y 组
            chosen_d_rot = dy;
            // 【核心修正】：让 X 组强行对齐 Y 组
            m_x_correction_m += (dy - dx);
            LOG_WARNING("X轴疑似打滑，已向Y轴对齐。");
        }
    }
    else
    {
        // 无明显打滑，取平均值以消除微小的机械测量误差
        chosen_d_rot = (dx + dy) / 2.0f;
    }

    // 4. 将选定的位移增量转换为角度增量并累加
    // 旋转弧长 s = 2 * L * theta (因为是两组轮子加和，所以是 2 * L)
    // 故 theta_inc = s_inc / (2 * L)
    float d_theta = chosen_d_rot / (2.0f * m_cfg.wheelbase);
    m_now_angle += d_theta;

    // 5. 角度归一化处理 [-PI, PI]
    while (m_now_angle > PI)
        m_now_angle -= 2.0f * PI;
    while (m_now_angle < -PI)
        m_now_angle += 2.0f * PI;

    // 6. 保存当前状态供下一帧计算增量
    m_last_shared_rot_m += chosen_d_rot;
}

void Chassis::chassis_init(Class_Motor_DJI_C620 &x_p, Class_Motor_DJI_C620 &x_m, 
                          Class_Motor_DJI_C620 &y_p, Class_Motor_DJI_C620 &y_m, 
                          WorldPosition::Config* cfg_in) {
    m_motors[0] = &x_p;
    m_motors[1] = &x_m;
    m_motors[2] = &y_p;
    m_motors[3] = &y_m;

    WorldPosition::Config cfg;
    if (cfg_in == nullptr) {
        cfg.ppr = 8192.0f;
        cfg.ratio = 3591.0f / 187.0f;
        cfg.wheel_radius = 0.076f;
        cfg.wheelbase = 0.25f;
    } else {
        cfg = *cfg_in;
    }

    m_worldPosition.init(
        &x_p.Get_Rx_Data().Total_Encoder, &x_m.Get_Rx_Data().Total_Encoder,
        &y_p.Get_Rx_Data().Total_Encoder, &y_m.Get_Rx_Data().Total_Encoder,
        cfg);

    m_radius = cfg.wheel_radius;
    m_gear_ratio = cfg.ratio;
    m_vel_to_omega_coeff = (1.0f / m_radius) * m_gear_ratio;
}

/**
 * @brief 1ms 定时器回调函数：执行实际计算
 */
void Chassis::TIM_1ms_Calculate_PeriodElapsedCallback() {
    // 1. 打滑检测与航位推算更新
    if(!m_target_vx && !m_target_vy && !m_target_vw) return;

    m_worldPosition.update();    

    float exec_vx = m_target_vx;
    float exec_vy = m_target_vy;

    // 2. 世界坐标系处理（含相位延迟补偿）
    if (m_is_world_frame) {
        // 获取当前角度
        float angle_now = m_worldPosition.getAngle();
        
        // 计算当前底盘真实旋转角速度 (用于预测)
        // 逻辑：平均各电机反馈速度并消除减速比影响
        float real_omega_w = (m_motors[0]->Get_Now_Omega() + m_motors[1]->Get_Now_Omega() + 
                              m_motors[2]->Get_Now_Omega() + m_motors[3]->Get_Now_Omega()) 
                              / (4.0f * m_gear_ratio); 
        
        // 预测角度 = 当前角度 + 角速度 * 预期延迟时间
        float angle_predict = angle_now + real_omega_w * (m_delay_comp_ms * 0.001f);

        float cos_a, sin_a;
        sin_a = arm_sin_f32(angle_predict);
        cos_a = arm_cos_f32(angle_predict);

        // 旋转矩阵变换
        exec_vx = m_target_vx * cos_a + m_target_vy * sin_a;
        exec_vy = -m_target_vx * sin_a + m_target_vy * cos_a;
    }

    // 3. 逆运动学解算 (计算各电机线速度)
    float motor_v[4];
    motor_v[0] =  exec_vx + m_target_vw; // X+
    motor_v[1] = -exec_vx + m_target_vw; // X-
    motor_v[2] =  exec_vy + m_target_vw; // Y+
    motor_v[3] = -exec_vy + m_target_vw; // Y-

    // 4. 速度缩放保护 (防止超限畸变)
    const float max_v = 3.5f; 
    float current_max = 0.0f;
    for (int i = 0; i < 4; i++) {
        float abs_v = fabsf(motor_v[i]);
        if (abs_v > current_max) current_max = abs_v;
    }

    float scale = (current_max > max_v) ? (max_v / current_max) : 1.0f;

    // 5. 最终目标转换并下发至电机驱动
    for (int i = 0; i < 4; i++) {
        m_motors[i]->Set_Target_Omega(motor_v[i] * scale * m_vel_to_omega_coeff);
    }
}
