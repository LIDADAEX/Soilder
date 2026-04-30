/**
 * @brief PID 算法模块
 * @note 采用 m_ 前缀标识成员变量，优化变速积分与前馈逻辑
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief PID 参数初始化
 * @param K_P P项系数
 * @param K_I I项系数
 * @param K_D D项系数
 * @param K_F 前馈系数
 * @param I_Out_Max 积分输出限幅
 * @param Out_Max 总输出限幅
 * @param D_T 计算周期 (s)
 * @param Dead_Zone 死区控制阈值
 * @param I_Variable_Speed_A 变速积分起始阈值
 * @param I_Variable_Speed_B 变速积分截止阈值
 * @param I_Separate_Threshold 积分分离阈值
 * @param D_First 是否开启微分先行
 */
void Class_PID::Init(float K_P, float K_I, float K_D, float K_F,
                     float I_Out_Max, float Out_Max, float D_T,
                     float Dead_Zone, float I_Variable_Speed_A,
                     float I_Variable_Speed_B, float I_Separate_Threshold,
                     Enum_PID_D_First D_First) {
    m_K_P = K_P;
    m_K_I = K_I;
    m_K_D = K_D;
    m_K_F = K_F;
    m_I_Out_Max = I_Out_Max;
    m_Out_Max = Out_Max;
    m_D_T = D_T;
    m_Dead_Zone = Dead_Zone;
    m_I_Variable_Speed_A = I_Variable_Speed_A;
    m_I_Variable_Speed_B = I_Variable_Speed_B;
    m_I_Separate_Threshold = I_Separate_Threshold;
    m_D_First = D_First;

    // 清空历史状态，确保冷启动安全
    m_Integral_Error = 0.0f;
    m_Pre_Error = 0.0f;
    m_Pre_Now = 0.0f;
    m_Pre_Target = 0.0f;
    m_Out = 0.0f;
}

/**
 * @brief PID 计算周期回调
 * @note 建议在定时器中断或固定周期任务中调用
 */
void Class_PID::TIM_Calculate_PeriodElapsedCallback() {
    float p_out, i_out, d_out, f_out;
    float error, abs_error;
    float speed_ratio = 1.0f;

    // 计算当前误差
    error = m_Target - m_Now;
    abs_error = Math_Abs(error);

    // 1. 死区处理
    if (abs_error < m_Dead_Zone) {
        error = 0.0f;
        abs_error = 0.0f;
    }

    // 2. P 项计算
    p_out = m_K_P * error;

    // 3. I 项计算 (包含变速积分与积分分离逻辑)
    // 变速积分比例计算
    if (m_I_Variable_Speed_B > m_I_Variable_Speed_A) {
        if (abs_error <= m_I_Variable_Speed_A) {
            speed_ratio = 1.0f;
        } else if (abs_error >= m_I_Variable_Speed_B) {
            speed_ratio = 0.0f;
        } else {
            speed_ratio = (m_I_Variable_Speed_B - abs_error) / (m_I_Variable_Speed_B - m_I_Variable_Speed_A);
        }
    }

    // 积分分离逻辑：若误差过大，直接清空积分，防止严重超调
    if (m_I_Separate_Threshold != 0.0f && abs_error >= m_I_Separate_Threshold) {
        m_Integral_Error = 0.0f;
    } else {
        m_Integral_Error += speed_ratio * error * m_D_T;
        // 积分抗饱和限制 (Anti-Windup)
        if (m_I_Out_Max != 0.0f) {
            float i_limit = m_I_Out_Max / m_K_I;
            Math_Constrain(&m_Integral_Error, -i_limit, i_limit);
        }
    }
    i_out = m_K_I * m_Integral_Error;

    // 4. D 项计算 (标准微分 vs 微分先行)
    if (m_D_First == PID_D_First_DISABLE) {
        // 标准微分：基于误差变化，响应快但目标突变时有冲击
        d_out = m_K_D * (error - m_Pre_Error) / m_D_T;
    } else {
        // 微分先行：基于测量值变化，输出平滑，适合目标频繁跳变的系统
        d_out = -m_K_D * (m_Now - m_Pre_Now) / m_D_T;
    }

    // 5. 前馈补偿计算 (Feed-Forward)
    // 采用静态前馈，用于补偿系统的基础负载（如重力、静摩擦等）
    f_out = m_K_F * m_Target;

    // 6. 汇总输出与总输出限幅
    m_Out = p_out + i_out + d_out + f_out;
    if (m_Out_Max != 0.0f) {
        Math_Constrain(&m_Out, -m_Out_Max, m_Out_Max);
    }

    // 7. 更新历史记录供下次计算使用
    m_Pre_Error = error;
    m_Pre_Now = m_Now;
    m_Pre_Target = m_Target;
}