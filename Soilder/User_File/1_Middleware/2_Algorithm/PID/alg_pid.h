#pragma once

/**
 * @brief PID 算法模块头文件
 * @note 统一采用 m_ 成员变量风格，支持高级积分补偿与微分先行
 */

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/Math/drv_math.h" // 确保路径正确

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行枚举
 */
enum Enum_PID_D_First {
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

/**
 * @brief PID 算法类
 */
class Class_PID {
public:
    /**
     * @brief 初始化 PID 参数
     */
    void Init(float K_P, float K_I, float K_D, float K_F = 0.0f,
              float I_Out_Max = 0.0f, float Out_Max = 0.0f, float D_T = 0.001f,
              float Dead_Zone = 0.0f, float I_Variable_Speed_A = 0.0f,
              float I_Variable_Speed_B = 0.0f, float I_Separate_Threshold = 0.0f,
              Enum_PID_D_First D_First = PID_D_First_DISABLE);

    /**
     * @brief 计算周期回调
     */
    void TIM_Calculate_PeriodElapsedCallback();

    /* Setter & Getter (Inline) ----------------------------------------------*/

    // 目标值与当前值
    void Set_Target(float Target) { m_Target = Target; }
    void Set_Now(float Now) { m_Now = Now; }
    float Get_Out() const { return m_Out; }

    // PID 系数
    void Set_K_P(float K_P) { m_K_P = K_P; }
    void Set_K_I(float K_I) { m_K_I = K_I; }
    void Set_K_D(float K_D) { m_K_D = K_D; }
    void Set_K_F(float K_F) { m_K_F = K_F; }
    float Get_K_P() const { return m_K_P; }
    float Get_K_I() const { return m_K_I; }
    float Get_K_D() const { return m_K_D; }
    float Get_K_F() const { return m_K_F; }

    // 限幅与死区
    void Set_I_Out_Max(float I_Out_Max) { m_I_Out_Max = I_Out_Max; }
    void Set_Out_Max(float Out_Max) { m_Out_Max = Out_Max; }
    void Set_Dead_Zone(float Dead_Zone) { m_Dead_Zone = Dead_Zone; }
    float Get_I_Out_Max() const { return m_I_Out_Max; }
    float Get_Out_Max() const { return m_Out_Max; }
    float Get_Dead_Zone() const { return m_Dead_Zone; }

    // 变速积分与积分分离
    void Set_I_Variable_Speed_A(float A) { m_I_Variable_Speed_A = A; }
    void Set_I_Variable_Speed_B(float B) { m_I_Variable_Speed_B = B; }
    void Set_I_Separate_Threshold(float Thr) { m_I_Separate_Threshold = Thr; }
    float Get_I_Variable_Speed_A() const { return m_I_Variable_Speed_A; }
    float Get_I_Variable_Speed_B() const { return m_I_Variable_Speed_B; }
    float Get_I_Separate_Threshold() const { return m_I_Separate_Threshold; }

    // 积分值操作
    void Set_Integral_Error(float Error) { m_Integral_Error = Error; }
    float Get_Integral_Error() const { return m_Integral_Error; }

private:
    // PID 参数成员
    float m_K_P = 0.0f;
    float m_K_I = 0.0f;
    float m_K_D = 0.0f;
    float m_K_F = 0.0f;

    // 配置参数
    float m_D_T = 0.001f;                 // 计算周期 (s)
    float m_Dead_Zone = 0.0f;             // 死区
    float m_I_Out_Max = 0.0f;             // 积分限幅
    float m_Out_Max = 0.0f;               // 输出限幅
    float m_I_Variable_Speed_A = 0.0f;    // 变速积分 A
    float m_I_Variable_Speed_B = 0.0f;    // 变速积分 B
    float m_I_Separate_Threshold = 0.0f;  // 积分分离阈值
    Enum_PID_D_First m_D_First;           // 微分先行模式

    // 运行时状态变量
    float m_Target = 0.0f;                // 目标值
    float m_Now = 0.0f;                   // 当前测量值
    float m_Out = 0.0f;                   // 输出值
    float m_Integral_Error = 0.0f;        // 累计积分值
    
    float m_Pre_Now = 0.0f;               // 上次测量值
    float m_Pre_Target = 0.0f;            // 上次目标值
    float m_Pre_Error = 0.0f;             // 上次误差值
};