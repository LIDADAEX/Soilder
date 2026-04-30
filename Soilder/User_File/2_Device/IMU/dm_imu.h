#pragma once

/**
 * @brief 达妙 IMU 驱动模块
 * @note 支持加速度、角速度、欧拉角及四元数解析，集成一阶低通滤波
 */

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include <string.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IMU 寄存器/命令定义
 */
enum Enum_IMU_Reg_ID : uint8_t {
    IMU_REG_REBOOT          = 0,
    IMU_REG_ACCEL           = 1,
    IMU_REG_GYRO            = 2,
    IMU_REG_EULER           = 3,
    IMU_REG_QUAT            = 4,
    IMU_REG_SET_ZERO        = 5,
    IMU_REG_CHANGE_ACTIVE   = 11,
    IMU_REG_SAVE_PARAM      = 254,
    IMU_REG_RESTORE_SETTING = 255
};

/**
 * @brief 一阶低通滤波器
 */
struct Struct_Low_Pass_Filter {
    float m_Alpha; 
    float m_Out_Prev;

    void Init(float Alpha) {
        m_Alpha = Alpha;
        m_Out_Prev = 0.0f;
    }

    float Update(float Input) {
        float output = m_Alpha * Input + (1.0f - m_Alpha) * m_Out_Prev;
        m_Out_Prev = output;
        return output;
    }
};

/**
 * @brief IMU 数据存储结构体
 */
struct Struct_IMU_Data {
    // 姿态数据
    float Accel[3];
    float Gyro[3];
    float Pitch, Roll, Yaw;
    float Quat[4];

    // 滤波后数据
    float Accel_Filtered[3];
    float Gyro_Filtered[3];
};

/* Class Definition ----------------------------------------------------------*/

class Class_IMU {
public:
    void Init(CAN_HandleTypeDef* hcan, uint16_t CAN_ID, float Alpha_Accel = 0.3f, float Alpha_Gyro = 0.5f);

    // 指令接口
    void Write_Reg(Enum_IMU_Reg_ID Reg_ID, uint32_t Data);
    void Read_Reg(Enum_IMU_Reg_ID Reg_ID);
    void Set_Active_Mode(bool Active);
    void Save_Config();

    // 回调函数
    void CAN_RxCpltCallback(uint8_t* Rx_Data);
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    // Getters
    const Struct_IMU_Data& Get_IMU_Data() const { return m_Data; }
    float Get_Pitch() const { return m_Data.Pitch; }
    float Get_Roll()  const { return m_Data.Roll; }
    float Get_Yaw()   const { return m_Data.Yaw; }

private:
    // 硬件绑定
    CAN_HandleTypeDef* m_hcan;
    uint16_t m_CAN_ID;

    // 数据与滤波
    Struct_IMU_Data m_Data;
    Struct_Low_Pass_Filter m_LPF_Accel[3];
    Struct_Low_Pass_Filter m_LPF_Gyro[3];

    // 内部私有方法
    void Send_Cmd(Enum_IMU_Reg_ID Reg_ID, uint8_t Is_Write, uint32_t Data);
    
    // 协议转换工具
    static inline float Math_Uint_To_Float(uint16_t x_ints, float x_min, float x_max, uint8_t bits) {
        float span = x_max - x_min;
        return ((float)x_ints) * span / ((float)((1 << bits) - 1)) + x_min;
    }
};