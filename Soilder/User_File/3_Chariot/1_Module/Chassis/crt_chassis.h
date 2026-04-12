/**
 * @file crt_chassis.h
 * @brief 底盘控制与航位推算系统头文件
 * @details 包含 WorldPosition (位置/打滑监控) 和 Chassis (底盘运动学) 两个核心类
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "1_Middleware/3_Debug/debug_log.h"

/* ========================================================================= */
/* =                         1. WorldPosition 类                           = */
/* ========================================================================= */

/**
 * @brief 基于全向轮组反馈的航位推算类
 * @note 采用双组冗余校验逻辑，能自动识别并补偿单边打滑
 */
class WorldPosition
{
public:
    /** @brief 物理配置参数 */
    struct Config
    {
        float ppr;          // 编码器单圈脉冲数 (通常 DJI 电机为 8192)
        float ratio;        // 减速比 (如 1:19 则填 19.2)
        float wheel_radius; // 轮子半径 (米)
        float wheelbase;    // 旋转中心到轮子的力臂 (米)
    };

    /**
     * @brief 初始化定位器
     * @param x_p, x_m, y_p, y_m 指向电机反馈的总脉冲计数的指针
     */
    void init(const int32_t *x_p, const int32_t *x_m,
              const int32_t *y_p, const int32_t *y_m,
              Config cfg);

    /** @brief 同步当前物理读数，重置计算原点（防止电机重启带来的跳变） */
    void reset_origin();

    /** @brief 增量更新计算：需在固定频率(如 1ms)下调用 */
    void update();

    // --- 数据获取 ---
    inline float getAngle() const { return m_now_angle; }
    inline float getAngleDegrees() const { return m_now_angle * 180.0f / PI; }

private:
    // 外部数据源指针
    const int32_t *m_x_p_ptr, *m_x_m_ptr, *m_y_p_ptr, *m_y_m_ptr;

    Config m_cfg;
    float  m_tick_to_dist; // 脉冲转位移系数

    // 状态量记录
    float m_last_shared_rot_m; // 上一时刻的平均参考位移
    float m_x_correction_m;    // X组累计修正偏移
    float m_y_correction_m;    // Y组累计修正偏移
    float m_now_angle;         // 当前累计角度 (弧度)
};

/* ========================================================================= */
/* =                            2. Chassis 类                              = */
/* ========================================================================= */

/**
 * @brief 底盘控制类：处理运动学逆解与世界坐标系转换
 */
class Chassis {
public:
    // --- 控制目标获取 (Getters) ---
    inline float Get_Target_VX() const { return m_target_vx; }
    inline float Get_Target_VY() const { return m_target_vy; }
    inline float Get_Target_VW() const { return m_target_vw; }
    inline bool  Get_World_Frame_Status() const { return m_is_world_frame; }
    inline float Get_Delay_Comp_Ms() const { return m_delay_comp_ms; }
    
    // --- 控制目标设置 (Setters) ---
    inline void Set_Target_VX(float v_x) { m_target_vx = v_x; }
    inline void Set_Target_VY(float v_y) { m_target_vy = v_y; }
    inline void Set_Target_VW(float v_w) { m_target_vw = v_w; }
    inline void Set_World_Frame_Status(bool status) { m_is_world_frame = status; }
    inline void Set_Delay_Comp_Ms(float ms) { m_delay_comp_ms = ms; }

    /**
     * @brief 一次性设置所有控制目标
     */
    void Set_Control_Target(float v_x, float v_y, float v_w, bool is_world_frame) {
        m_target_vx = v_x;
        m_target_vy = v_y;
        m_target_vw = v_w;
        m_is_world_frame = is_world_frame;
    }

    /**
     * @brief 底盘初始化
     * @param cfg_in 若为 nullptr 则使用默认硬件参数
     */
    void chassis_init(Class_Motor_DJI_C620 &x_p, Class_Motor_DJI_C620 &x_m, 
                      Class_Motor_DJI_C620 &y_p, Class_Motor_DJI_C620 &y_m, 
                      WorldPosition::Config* cfg_in = nullptr);

    /**
     * @brief 控制主循环：需在 1ms 定时器回调中调用
     * @details 流程：状态检查 -> 位置更新 -> 坐标变换 -> 逆运动学解算 -> 线速度限幅 -> 电机控制输出
     */
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    /**
     * @brief 健康检查循环：建议在 100ms 定时器中调用
     */
    void TIM_100ms_Alive_PeriodElapsedCallback();

    // 定位实例暴露给外部（用于调试或上层导航）
    WorldPosition m_worldPosition;

private:
    // 内部电机指针关联数组 (0:X+, 1:X-, 2:Y+, 3:Y-)
    Class_Motor_DJI_C620* m_motors[4];
    
    // 控制状态量
    float m_target_vx = 0.0f;
    float m_target_vy = 0.0f;
    float m_target_vw = 0.0f;
    bool  m_is_world_frame = false;

    // 预计算物理系数
    float m_radius;
    float m_gear_ratio;
    float m_vel_to_omega_coeff; // 速度到转速的映射比
    
    // 系统延迟补偿参数 (经验值: 2.0ms)
    float m_delay_comp_ms = 2.0f; 
};

#endif // CRT_CHASSIS_H