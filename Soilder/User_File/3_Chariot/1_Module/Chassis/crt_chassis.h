#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "1_Middleware/3_Debug/debug_log.h"

class WorldPosition
{
public:
    // 硬件参数结构体
    struct Config
    {
        float ppr;          // 编码器单圈脉冲数 (Pulses Per Revolution)
        float ratio;        // 减速比 (如 1:30 则填 30)
        float wheel_radius; // 轮子半径 (米)
        float wheelbase;    // 旋转中心到轮子的力臂 (米)
    };

    /**
     * @brief 初始化底盘驱动
     * @param x_p, x_m, y_p, y_m 编码器总脉冲指针 (假设顺时针旋转时同向增加)
     * @param cfg 物理参数配置
     */
    void init(const int32_t *x_p, const int32_t *x_m,
              const int32_t *y_p, const int32_t *y_m,
              Config cfg);

    /**
     * @brief 角度更新计算（建议在固定频率定时器中调用）
     */
    void update();

    float getAngle() const { return m_now_angle; }
    float getAngleDegrees() const { return m_now_angle * 180.0f / PI; }

private:
    // 外部编码器指针
    const int32_t *m_x_p_ptr, *m_x_m_ptr, *m_y_p_ptr, *m_y_m_ptr;

    // 硬件转换系数
    Config m_cfg;
    float m_tick_to_dist;

    // 状态记录

    float m_last_shared_rot_m; // 上一时刻的旋转贡献总量
    float m_x_correction_m;    // X 组的累计修正位移（米）
    float m_y_correction_m;    // Y 组的累计修正位移（米）

    float m_now_angle; // 当前绝对角度（弧度）
};

class Chassis {
public:
    // --- Get 接口 ---
    inline float Get_Target_VX() const { return m_target_vx; }
    inline float Get_Target_VY() const { return m_target_vy; }
    inline float Get_Target_VW() const { return m_target_vw; }
    inline bool  Get_World_Frame_Status() const { return m_is_world_frame; }
    inline float Get_Delay_Comp_Ms() const { return m_delay_comp_ms; }
    
    // --- Set 接口 ---
    inline void Set_Target_VX(float v_x) { m_target_vx = v_x; }
    inline void Set_Target_VY(float v_y) { m_target_vy = v_y; }
    inline void Set_Target_VW(float v_w) { m_target_vw = v_w; }
    inline void Set_World_Frame_Status(bool status) { m_is_world_frame = status; }
    inline void Set_Delay_Comp_Ms(float ms) { m_delay_comp_ms = ms; }

    /**
     * @brief 底盘初始化
     */
    void chassis_init(Class_Motor_DJI_C620 &x_p, Class_Motor_DJI_C620 &x_m, 
                      Class_Motor_DJI_C620 &y_p, Class_Motor_DJI_C620 &y_m, 
                      WorldPosition::Config* cfg_in = nullptr);

    /**
     * @brief 设置底盘控制目标（仅修改成员变量，不进行计算）
     * @param v_x, v_y, v_w 期望物理值 (m/s, rad/s)
     * @param is_world_frame 是否开启世界坐标系控制
     */
    // 原有的控制设置接口保持，方便一次性下发
    void Set_Control_Target(float v_x, float v_y, float v_w, bool is_world_frame) {
        m_target_vx = v_x;
        m_target_vy = v_y;
        m_target_vw = v_w;
        m_is_world_frame = is_world_frame;
    }

    /**
     * @brief 核心执行函数：需在 1ms 定时器中断中调用
     * 包含：角度更新、相位补偿、坐标变换、逆运动学、速度缩放、电机下发
     */
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    // 暴露航位推算对象
    WorldPosition m_worldPosition;

private:
    // 电机实例指针
    Class_Motor_DJI_C620* m_motors[4];
    
    // 控制目标成员变量
    float m_target_vx = 0.0f;
    float m_target_vy = 0.0f;
    float m_target_vw = 0.0f;
    bool  m_is_world_frame = false;

    // 物理参数与系数
    float m_radius;
    float m_gear_ratio;
    float m_vel_to_omega_coeff;
    
    // 补偿参数 (1.5ms - 3.0ms 是经验值，可根据总线延迟微调)
    float m_delay_comp_ms = 2.0f; 
};
#endif
