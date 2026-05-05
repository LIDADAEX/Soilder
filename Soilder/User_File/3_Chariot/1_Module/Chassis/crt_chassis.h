#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "1_Middleware/3_Debug/debug_log.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/IMU/dm_imu.h"
#include "2_Device/IMU_board/BMI088.h"
#include "2_Device/IMU_board/IST8310.h"

#include "arm_math.h"  // 引入 ARM DSP 库

extern Class_Motor_DJI_C620 motor_x_p;
extern Class_Motor_DJI_C620 motor_x_m;
extern Class_Motor_DJI_C620 motor_y_p;
extern Class_Motor_DJI_C620 motor_y_m;

class WorldPosition {
   public:
    struct Config {
        float32_t ppr;
        float32_t ratio;
        float32_t wheel_radius;
        float32_t wheelbase;
    };

    void init(const int32_t* x_p, const int32_t* x_m, const int32_t* y_p, const int32_t* y_m, Config cfg);

    void reset_origin();
    void update();

    // --- 数据获取接口 ---
    inline float32_t getAngle() const { return m_now_angle; }
    inline float32_t getXCorrection() const { return m_x_correction_m; }
    inline float32_t getYCorrection() const { return m_y_correction_m; }
    void getRawTicks(int32_t ticks[4]) const {
        ticks[0] = *m_x_p_ptr;
        ticks[1] = *m_x_m_ptr;
        ticks[2] = *m_y_p_ptr;
        ticks[3] = *m_y_m_ptr;
    }

   private:
    const int32_t *m_x_p_ptr, *m_x_m_ptr, *m_y_p_ptr, *m_y_m_ptr;
    Config m_cfg;
    float32_t m_tick_to_dist;
    float32_t m_last_shared_rot_m;
    float32_t m_x_correction_m;
    float32_t m_y_correction_m;
    float32_t m_now_angle;

    enum SlipStatus { NO_SLIP = 0, X_SLIPPING, Y_SLIPPING } m_slip_state = NO_SLIP;
};

class Chassis {
   public:
    // --- 控制目标获取 (Getters) ---
    inline float32_t Get_Target_VX() const { return m_target_vx; }
    inline float32_t Get_Target_VY() const { return m_target_vy; }
    inline float32_t Get_Target_VW() const { return m_target_vw; }
    inline bool Get_World_Frame_Status() const { return m_is_world_frame; }
    inline float32_t Get_Delay_Comp_Ms() const { return m_delay_comp_ms; }
    inline float32_t Get_Deadzone() const { return m_deadzone; }
    inline float32_t Get_IMU_Paw() const {return m_IMU.Get_IMU_Data().Yaw; }
	inline float32_t Get_Chassis_Angle() const {return m_now_angle;}

    // --- 控制目标设置 (Setters) ---
    inline void Set_Target_VX(float32_t v_x) { m_target_vx = v_x; }
    inline void Set_Target_VY(float32_t v_y) { m_target_vy = v_y; }
    inline void Set_Target_VW(float32_t v_w) { m_target_vw = v_w; }
    inline void Set_World_Frame_Status(bool status) { m_is_world_frame = status; }
    inline void Set_Delay_Comp_Ms(float32_t ms) { m_delay_comp_ms = ms; }
    inline void Set_Deadzone(float32_t dz) { m_deadzone = dz; }

    /**
     * @brief 报错补全：一次性设置所有控制目标
     */
    void Set_Control_Target(float32_t v_x, float32_t v_y, float32_t v_w, bool is_world_frame) {
        m_target_vx = v_x;
        m_target_vy = v_y;
        m_target_vw = v_w;
        m_is_world_frame = is_world_frame;
    }

    /**
     * @brief 底盘初始化
     */
    void chassis_init(Class_Motor_DJI_C620& x_p,
                      Class_Motor_DJI_C620& x_m,
                      Class_Motor_DJI_C620& y_p,
                      Class_Motor_DJI_C620& y_m,
                      WorldPosition::Config* cfg_in = nullptr);

    /**
     * @brief 控制主循环 (1ms)
     */
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    /**
     * @brief 健康检查循环 (100ms)
     */
    void TIM_100ms_Alive_PeriodElapsedCallback();

    // 定位实例暴露给外部
    WorldPosition m_worldPosition;

    Class_IMU m_IMU;
    BMI088 m_IMU_Board;
    IST8310 m_IST8310;

   private:
    // 内部电机指针关联数组 (0:X+, 1:X-, 2:Y+, 3:Y-)
    Class_Motor_DJI_C620* m_motors[4];

    // 控制状态量 (统一使用 float32_t)
    float32_t m_target_vx = 0.0f;
    float32_t m_target_vy = 0.0f;
    float32_t m_target_vw = 0.0f;
    float32_t m_deadzone = 0.02f;
	float32_t m_now_angle = 0.00f;
    bool m_is_world_frame = false;

    // 预计算物理系数
    float32_t m_radius;
    float32_t m_gear_ratio;
    float32_t m_vel_to_omega_coeff;

    // 系统延迟补偿参数
    float32_t m_delay_comp_ms = 2.0f;
};

#endif