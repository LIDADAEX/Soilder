/**
 * @file dvc_motor_dji.h
 * @brief 大疆电机配置与操作 - 优化版
 * @version 1.2
 * @date 2024-03-07
 * @copyright USTC-RoboWalker (c) 2023-2024
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"

/* Exported types ------------------------------------------------------------*/

enum Enum_Motor_DJI_Status {
    Motor_DJI_Status_DISABLE = 0,
    Motor_DJI_Status_ENABLE,
};

enum Enum_Motor_DJI_ID {
    Motor_DJI_ID_0x201 = 1, Motor_DJI_ID_0x202, Motor_DJI_ID_0x203, Motor_DJI_ID_0x204,
    Motor_DJI_ID_0x205, Motor_DJI_ID_0x206, Motor_DJI_ID_0x207, Motor_DJI_ID_0x208,
    Motor_DJI_ID_0x209, Motor_DJI_ID_0x20A, Motor_DJI_ID_0x20B,
};

enum Enum_Motor_DJI_Control_Method {
    Motor_DJI_Control_Method_VOLTAGE = 0,
    Motor_DJI_Control_Method_CURRENT,
    Motor_DJI_Control_Method_TORQUE,
    Motor_DJI_Control_Method_OMEGA,
    Motor_DJI_Control_Method_ANGLE,
};

enum Enum_Motor_DJI_GM6020_Driver_Version {
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,
    Motor_DJI_GM6020_Driver_Version_2023,
};

enum Enum_Motor_DJI_Power_Limit_Status {
    Motor_DJI_Power_Limit_Status_DISABLE = 0,
    Motor_DJI_Power_Limit_Status_ENABLE,
};

#pragma pack(push, 1)
struct Struct_Motor_DJI_CAN_Rx_Data {
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
};
#pragma pack(pop)

struct Struct_Motor_DJI_Rx_Data {
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_Temperature;
    float Now_Power;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/* Classes -------------------------------------------------------------------*/

/**
 * @brief GM6020无刷电机类
 */
class Class_Motor_DJI_GM6020 {
public:
    Class_PID PID_Angle;
    Class_PID PID_Omega;
    Class_PID PID_Current;

    static const uint16_t Encoder_Num_Per_Round = 8192;

    void Init(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_rx_id,
              Enum_Motor_DJI_Control_Method control_method = Motor_DJI_Control_Method_ANGLE,
              int32_t encoder_offset = 0,
              Enum_Motor_DJI_GM6020_Driver_Version drive_version = Motor_DJI_GM6020_Driver_Version_DEFAULT,
              Enum_Motor_DJI_Power_Limit_Status power_limit_status = Motor_DJI_Power_Limit_Status_DISABLE,
              float voltage_max = 24.0f, float current_max = 3.0f);

    // 内联函数类内实现[cite: 2]
    float Get_Voltage_Max() { return Voltage_Max; }
    float Get_Current_Max() { return Current_Max; }
    float Get_Power_K_0() { return Power_K_0; }
    float Get_Power_K_1() { return Power_K_1; }
    float Get_Power_K_2() { return Power_K_2; }
    float Get_Power_A() { return Power_A; }
    float Get_Theoretical_Output_Voltage_Max() { return Theoretical_Output_Voltage_Max; }
    float Get_Theoretical_Output_Current_Max() { return Theoretical_Output_Current_Max; }
    Enum_Motor_DJI_Status Get_Status() { return Motor_DJI_Status; }
    float Get_Now_Angle() { return Rx_Data.Now_Angle; }
    float Get_Now_Omega() { return Rx_Data.Now_Omega; }
    float Get_Now_Current() { return Rx_Data.Now_Current; }
    float Get_Now_Temperature() { return Rx_Data.Now_Temperature; }
    float Get_Now_Power() { return Rx_Data.Now_Power; }
    float Get_Power_Estimate() { return Power_Estimate; }
    Enum_Motor_DJI_Control_Method Get_Control_Method() { return Motor_DJI_Control_Method; }
    float Get_Target_Angle() { return Target_Angle; }
    float Get_Target_Omega() { return Target_Omega; }
    float Get_Target_Current() { return Target_Current; }
    float Get_Target_Voltage() { return Target_Voltage; }
    float Get_Feedforward_Omega() { return Feedforward_Omega; }
    float Get_Feedforward_Current() { return Feedforward_Current; }
    float Get_Feedforward_Voltage() { return Feedforward_Voltage; }
    float Get_Power_Factor() { return Power_Factor; }

    void Set_Control_Method(Enum_Motor_DJI_Control_Method method) { Motor_DJI_Control_Method = method; }
    void Set_Target_Angle(float angle) { Target_Angle = angle; }
    void Set_Target_Omega(float omega) { Target_Omega = omega; }
    void Set_Target_Current(float current) { Target_Current = current; }
    void Set_Target_Voltage(float voltage) { Target_Voltage = voltage; }
    void Set_Feedforward_Omega(float omega) { Feedforward_Omega = omega; }
    void Set_Feedforward_Current(float current) { Feedforward_Current = current; }
    void Set_Feedforward_Voltage(float voltage) { Feedforward_Voltage = voltage; }
    void Set_Power_Factor(float factor) { Power_Factor = factor; }

    void CAN_RxCpltCallback(uint8_t* Rx_Data);
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object* CAN_Manage_Object;
    Enum_Motor_DJI_ID CAN_Rx_ID;
    uint8_t* Tx_Data;
    int32_t Encoder_Offset;
    Enum_Motor_DJI_GM6020_Driver_Version Driver_Version;
    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;

    float Voltage_Max, Current_Max;
    float Power_K_0 = 0.8130f, Power_K_1 = -0.0005f, Power_K_2 = 6.0021f, Power_A = 1.3715f;

    float Voltage_To_Out = 25000.0f / 24.0f;
    float Current_To_Out = 16384.0f / 3.0f;
    float Theoretical_Output_Voltage_Max = 24.0f;
    float Theoretical_Output_Current_Max = 3.0f;

    uint32_t Flag = 0, Pre_Flag = 0;
    float Out = 0.0f;
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    Struct_Motor_DJI_Rx_Data Rx_Data;
    float Power_Estimate;

    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    float Target_Angle = 0.0f, Target_Omega = 0.0f, Target_Current = 0.0f, Target_Voltage = 0.0f;
    float Feedforward_Omega = 0.0f, Feedforward_Current = 0.0f, Feedforward_Voltage = 0.0f;
    float Power_Factor = 1.0f;

    void Data_Process();
    void PID_Calculate();
    void Power_Limit_Control();
    void Output();
};

/**
 * @brief C610无刷电调类[cite: 2]
 */
class Class_Motor_DJI_C610 {
public:
    Class_PID PID_Angle;
    Class_PID PID_Omega;

    static const uint16_t Encoder_Num_Per_Round = 8192;

    void Init(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_rx_id,
              Enum_Motor_DJI_Control_Method control_method = Motor_DJI_Control_Method_OMEGA,
              float gearbox_rate = 36.0f, float current_max = 10.0f);

    float Get_Current_Max() { return Current_Max; }
    float Get_Theoretical_Output_Current_Max() { return Theoretical_Output_Current_Max; }
    Enum_Motor_DJI_Status Get_Status() { return Motor_DJI_Status; }
    float Get_Now_Angle() { return Rx_Data.Now_Angle; }
    float Get_Now_Omega() { return Rx_Data.Now_Omega; }
    float Get_Now_Current() { return Rx_Data.Now_Current; }
    uint8_t Get_Now_Temperature() { return (uint8_t)Rx_Data.Now_Temperature; }
    Enum_Motor_DJI_Control_Method Get_Control_Method() { return Motor_DJI_Control_Method; }
    float Get_Target_Angle() { return Target_Angle; }
    float Get_Target_Omega() { return Target_Omega; }
    float Get_Target_Current() { return Target_Current; }
    float Get_Feedforward_Omega() { return Feedforward_Omega; }
    float Get_Feedforward_Current() { return Feedforward_Current; }

    void Set_Control_Method(Enum_Motor_DJI_Control_Method method) { Motor_DJI_Control_Method = method; }
    void Set_Target_Angle(float angle) { Target_Angle = angle; }
    void Set_Target_Omega(float omega) { Target_Omega = omega; }
    void Set_Target_Current(float current) { Target_Current = current; }
    void Set_Feedforward_Omega(float omega) { Feedforward_Omega = omega; }
    void Set_Feedforward_Current(float current) { Feedforward_Current = current; }

    void CAN_RxCpltCallback(uint8_t* Rx_Data);
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object* CAN_Manage_Object;
    Enum_Motor_DJI_ID CAN_Rx_ID;
    uint8_t* Tx_Data;
    float Gearbox_Rate;
    float Current_Max;
    float Current_To_Out = 10000.0f / 10.0f;
    float Theoretical_Output_Current_Max = 10.0f;

    uint32_t Flag = 0, Pre_Flag = 0;
    float Out = 0.0f;
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    Struct_Motor_DJI_Rx_Data Rx_Data;

    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    float Target_Angle = 0.0f, Target_Omega = 0.0f, Target_Current = 0.0f;
    float Feedforward_Omega = 0.0f, Feedforward_Current = 0.0f;

    void Data_Process();
    void PID_Calculate();
    void Output();
};

/**
 * @brief C620无刷电调类[cite: 2]
 */
class Class_Motor_DJI_C620 {
public:
    Class_PID PID_Angle;
    Class_PID PID_Omega;

    static const uint16_t Encoder_Num_Per_Round = 8192;

    void Init(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_rx_id,
              Enum_Motor_DJI_Control_Method control_method = Motor_DJI_Control_Method_OMEGA,
              float gearbox_rate = 3591.0f / 187.0f,
              Enum_Motor_DJI_Power_Limit_Status power_limit_status = Motor_DJI_Power_Limit_Status_DISABLE,
              float current_max = 20.0f);

    float Get_Current_Max() { return Current_Max; }
    float Get_Power_K_0() { return Power_K_0; }
    float Get_Power_K_1() { return Power_K_1; }
    float Get_Power_K_2() { return Power_K_2; }
    float Get_Power_A() { return Power_A; }
    float Get_Theoretical_Output_Current_Max() { return Theoretical_Output_Current_Max; }
    Enum_Motor_DJI_Status Get_Status() { return Motor_DJI_Status; }
    float Get_Now_Angle() { return Rx_Data.Now_Angle; }
    float Get_Now_Omega() { return Rx_Data.Now_Omega; }
    float Get_Now_Current() { return Rx_Data.Now_Current; }
    float Get_Now_Temperature() { return Rx_Data.Now_Temperature; }
    float Get_Now_Power() { return Rx_Data.Now_Power; }
    float Get_Power_Estimate() { return Power_Estimate; }
    Enum_Motor_DJI_Control_Method Get_Control_Method() { return Motor_DJI_Control_Method; }
    float Get_Target_Angle() { return Target_Angle; }
    float Get_Target_Omega() { return Target_Omega; }
    float Get_Target_Current() { return Target_Current; }
    float Get_Feedforward_Omega() { return Feedforward_Omega; }
    float Get_Feedforward_Current() { return Feedforward_Current; }
    Struct_Motor_DJI_Rx_Data& Get_Rx_Data() { return Rx_Data; }
    float Get_Gearbox_Rate() { return Gearbox_Rate; }

    void Set_Control_Method(Enum_Motor_DJI_Control_Method method) { Motor_DJI_Control_Method = method; }
    void Set_Target_Angle(float angle) { Target_Angle = angle; }
    void Set_Target_Omega(float omega) { Target_Omega = omega; }
    void Set_Target_Current(float current) { Target_Current = current; }
    void Set_Feedforward_Omega(float omega) { Feedforward_Omega = omega; }
    void Set_Feedforward_Current(float current) { Feedforward_Current = current; }
    void Set_Power_Factor(float factor) { Power_Factor = factor; }

    void CAN_RxCpltCallback(uint8_t* Rx_Data);
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object* CAN_Manage_Object;
    Enum_Motor_DJI_ID CAN_Rx_ID;
    uint8_t* Tx_Data;
    float Gearbox_Rate;
    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    float Current_Max;

    float Power_K_0 = 0.01716874f, Power_K_1 = 0.00004319f, Power_K_2 = 0.12785662f, Power_A = 1.59865837f;
    
    float Current_To_Out = 16384.0f / 20.0f;
    float Theoretical_Output_Current_Max = 20.0f;

    uint32_t Flag = 0, Pre_Flag = 0;
    float Out = 0.0f;
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    Struct_Motor_DJI_Rx_Data Rx_Data;
    float Power_Estimate;

    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    float Target_Angle = 0.0f, Target_Omega = 0.0f, Target_Current = 0.0f;
    float Feedforward_Omega = 0.0f, Feedforward_Current = 0.0f;
    float Power_Factor = 1.0f;

    void Data_Process();
    void PID_Calculate();
    void Power_Limit_Control();
    void Output();
};