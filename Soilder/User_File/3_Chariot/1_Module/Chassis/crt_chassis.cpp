#include "crt_chassis.h"

Class_Motor_DJI_C620 motor_x_p;
Class_Motor_DJI_C620 motor_x_m;
Class_Motor_DJI_C620 motor_y_p;
Class_Motor_DJI_C620 motor_y_m;

/* ================== 1. WorldPosition 实现 ================== */

void WorldPosition::init(const int32_t* x_p, const int32_t* x_m, const int32_t* y_p, const int32_t* y_m, Config cfg) {
    m_x_p_ptr = x_p;
    m_x_m_ptr = x_m;
    m_y_p_ptr = y_p;
    m_y_m_ptr = y_m;
    m_cfg = cfg;

    // 距离系数：(2 * PI * r) / (ppr * ratio)
    m_tick_to_dist = (2.0f * PI * m_cfg.wheel_radius) / (m_cfg.ppr * m_cfg.ratio);

    m_x_correction_m = 0.0f;
    m_y_correction_m = 0.0f;
    m_now_angle = 0.0f;

    float32_t init_x_rot = (float32_t)(*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist;
    float32_t init_y_rot = (float32_t)(*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist;
    m_last_shared_rot_m = (init_x_rot + init_y_rot) * 0.5f;
}

void WorldPosition::reset_origin() {
    // 使用 DSP 库进行浮点转换
    float32_t cur_x = (float32_t)(*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist + m_x_correction_m;
    float32_t cur_y = (float32_t)(*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist + m_y_correction_m;
    m_last_shared_rot_m = (cur_x + cur_y) * 0.5f;
}

void WorldPosition::update() {
    float32_t cur_x = (float32_t)(*m_x_p_ptr + *m_x_m_ptr) * m_tick_to_dist + m_x_correction_m;
    float32_t cur_y = (float32_t)(*m_y_p_ptr + *m_y_m_ptr) * m_tick_to_dist + m_y_correction_m;

    float32_t dx = cur_x - m_last_shared_rot_m;
    float32_t dy = cur_y - m_last_shared_rot_m;

    float32_t abs_dx, abs_dy, abs_diff;
    arm_abs_f32(&dx, &abs_dx, 1);
    arm_abs_f32(&dy, &abs_dy, 1);

    float32_t diff = dx - dy;
    arm_abs_f32(&diff, &abs_diff, 1);

    float32_t chosen_d_rot = 0.0f;

    // 冗余逻辑与打滑边缘通报
    if (abs_diff > 0.0001f) {
        if (abs_dx <= abs_dy) {
            chosen_d_rot = dx;
            m_y_correction_m += diff;
            if (m_slip_state != Y_SLIPPING) {
                LOG_WARNING(
                    "Chassis: Y-Axis slip "
                    "detected. Syncing to X.");
                m_slip_state = Y_SLIPPING;
            }
        } else {
            chosen_d_rot = dy;
            m_x_correction_m -= diff;
            if (m_slip_state != X_SLIPPING) {
                LOG_WARNING(
                    "Chassis: X-Axis slip "
                    "detected. Syncing to Y.");
                m_slip_state = X_SLIPPING;
            }
        }
    } else {
        chosen_d_rot = (dx + dy) * 0.5f;
        if (m_slip_state != NO_SLIP) {
            LOG_INFO("Chassis: Slip recovered.");
            m_slip_state = NO_SLIP;
        }
    }

    // 角度更新
    m_now_angle += chosen_d_rot / (2.0f * m_cfg.wheelbase);

    // 角度归一化使用 CMSIS-DSP 逻辑或手动快速逻辑
    while (m_now_angle > PI) m_now_angle -= 2.0f * PI;
    while (m_now_angle < -PI) m_now_angle += 2.0f * PI;

    m_last_shared_rot_m += chosen_d_rot;
}

/* ================== 2. Chassis 实现 ================== */

void Chassis::chassis_init(Class_Motor_DJI_C620& x_p,
                           Class_Motor_DJI_C620& x_m,
                           Class_Motor_DJI_C620& y_p,
                           Class_Motor_DJI_C620& y_m,
                           WorldPosition::Config* cfg_in) {
    m_motors[0] = &x_p;
    m_motors[1] = &x_m;
    m_motors[2] = &y_p;
    m_motors[3] = &y_m;

    WorldPosition::Config cfg;
    if (cfg_in == nullptr) {
        // 使用 DJI C620 默认参数
        cfg.ppr = (float32_t)x_p.Encoder_Num_Per_Round;
        cfg.ratio = (float32_t)x_p.Get_Gearbox_Rate();
        cfg.wheel_radius = 0.076f;
        cfg.wheelbase = 0.25f;
    } else {
        cfg = *cfg_in;
    }

    // 初始化推算器
    m_worldPosition.init(&x_p.Get_Rx_Data().Total_Encoder,
                         &x_m.Get_Rx_Data().Total_Encoder,
                         &y_p.Get_Rx_Data().Total_Encoder,
                         &y_m.Get_Rx_Data().Total_Encoder,
                         cfg);

    m_radius = cfg.wheel_radius;
    m_gear_ratio = cfg.ratio;
    // 物理量转换系数：V -> Omega
    m_vel_to_omega_coeff = 1.0f / m_radius;

    m_IMU.Init(&hcan2, 0x05, 0.2f, 0.4f);
    HAL_Delay(1);
    m_IMU.Set_Active_Mode(false);

}

void Chassis::TIM_1ms_Calculate_PeriodElapsedCallback() {
    // // 1. 检查电机在线状态
    // bool all_motors_healthy = true;

    // // 1. 电机状态检查与 PID 计算
    // for (uint8_t i = 0; i < 4; i++) {
    //     if (m_motors[i]->Get_Status() != Motor_DJI_Status_ENABLE) {
    //         all_motors_healthy = false;
    //         break;
    //     }
    // }

    // // 安全策略：电机不全则重置原点并停止

    // if (!all_motors_healthy) {
    //     m_worldPosition.reset_origin();
    //     for (uint8_t i = 0; i < 4; i++) {
    //         m_motors[i]->Set_Target_Omega(0);
    //         m_motors[i]->TIM_Calculate_PeriodElapsedCallback();
    //         m_motors[i]->TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();
    //     }
    //     return;
    // }

    // 2. 使用 arm_abs_f32 处理死区
    float32_t abs_vx, abs_vy, abs_vw;
    arm_abs_f32(&m_target_vx, &abs_vx, 1);
    arm_abs_f32(&m_target_vy, &abs_vy, 1);
    arm_abs_f32(&m_target_vw, &abs_vw, 1);

    float32_t in_vx = (abs_vx < m_deadzone) ? 0.0f : m_target_vx;
    float32_t in_vy = (abs_vy < m_deadzone) ? 0.0f : m_target_vy;
    float32_t in_vw = (abs_vw < m_deadzone) ? 0.0f : m_target_vw;

    if (in_vx == 0.0f && in_vy == 0.0f && in_vw == 0.0f) {
        for (int i = 0; i < 4; i++){ 
			m_motors[i]->Set_Target_Omega(0);
			m_motors[i]->TIM_Calculate_PeriodElapsedCallback();
		}
        return;
    }

    // // 3. 更新航位推算
    // m_worldPosition.update();

    float32_t exec_vx = in_vx;
    float32_t exec_vy = in_vy;

    // 4. 世界坐标系变换
    if (m_is_world_frame) {
        // float32_t angle_now = m_worldPosition.getAngle();
        float32_t angle_now = m_IMU.Data.yaw /360 * 2 * PI;
        float32_t avg_omega = (m_motors[0]->Get_Now_Omega() + m_motors[1]->Get_Now_Omega() +
                               m_motors[2]->Get_Now_Omega() + m_motors[3]->Get_Now_Omega()) *
                              0.25f;

        float32_t angle_predict = angle_now + avg_omega * (m_delay_comp_ms * 0.001f);

        float32_t sin_a = arm_sin_f32(angle_predict);
        float32_t cos_a = arm_cos_f32(angle_predict);

        exec_vx = in_vx * cos_a + in_vy * sin_a;
        exec_vy = -in_vx * sin_a + in_vy * cos_a;
    }

    // 5. 逆运动学与限幅 (使用 DSP 绝对值)
    float32_t motor_v[4];
    motor_v[0] = exec_vx + in_vw;
    motor_v[1] = -exec_vx + in_vw;
    motor_v[2] = exec_vy + in_vw;
    motor_v[3] = -exec_vy + in_vw;

    float32_t current_max = 0.0f;
    for (int i = 0; i < 4; i++) {
        float32_t abs_v;
        arm_abs_f32(&motor_v[i], &abs_v, 1);
        if (abs_v > current_max)
            current_max = abs_v;
    }

    const float32_t max_v = 3.5f;
    float32_t scale = (current_max > max_v) ? (max_v / current_max) : 1.0f;

    for (int i = 0; i < 4; i++) {
        m_motors[i]->Set_Target_Omega(motor_v[i] * scale * m_vel_to_omega_coeff);
        m_motors[i]->TIM_Calculate_PeriodElapsedCallback();
    }
}

void Chassis::TIM_100ms_Alive_PeriodElapsedCallback() {
    static bool initialed = false;
    static uint8_t cnt[4] = {0};

    for (uint8_t i = 0; i < 4; i++) {
        if (cnt[i] >= 10)
            continue;

        m_motors[i]->TIM_100ms_Alive_PeriodElapsedCallback();

        if (m_motors[i]->Get_Status() != Motor_DJI_Status_ENABLE) {
            if (!initialed)
                return;

            cnt[i]++;
            LOG_WARNING("电机" + std::to_string(i) + "掉线，重连中: " + std::to_string(cnt[i]));
            if (cnt[i] >= 10)
                LOG_ERROR("电机" + std::to_string(i) + "彻底丢失");
        } else {
            initialed = true;
            cnt[i] = 0;  // 恢复在线，重置计数
        }
    }
}