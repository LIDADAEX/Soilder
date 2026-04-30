/**
 * @file dvc_motor_dji.cpp
 * @brief 大疆电机配置与操作 - 完整实现版
 * @version 1.2
 * @date 2024-03-07
 */

#include "dvc_motor_dji.h"

/* Helper Functions (Internal Linkage) ----------------------------------------*/

/**
 * @brief 内部函数：根据电调ID和硬件类型分配CAN发送缓冲区指针
 * @note 该函数通过数学偏移计算取代了冗长的 switch-case 逻辑
 */
static uint8_t* allocate_tx_data(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_id, Enum_Motor_DJI_GM6020_Driver_Version version = Motor_DJI_GM6020_Driver_Version_DEFAULT) {
    uint8_t* base_ptr = nullptr;
    int index = (can_id - 1) % 4; // 计算在 8 字节 CAN 帧中的起始位置 (0, 2, 4, 6)
    bool is_can1 = (hcan->Instance == CAN1);

    if (can_id <= Motor_DJI_ID_0x204) {
        // ID 0x201-0x204 对应标识符 0x200
        base_ptr = is_can1 ? CAN1_0x200_Tx_Data : CAN2_0x200_Tx_Data;
    } else if (can_id <= Motor_DJI_ID_0x208) {
        // ID 0x205-0x208 对应标识符 0x1FF 或 0x1FE (GM6020新固件)
        if (version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            base_ptr = is_can1 ? CAN1_0x1ff_Tx_Data : CAN2_0x1ff_Tx_Data;
        else
            base_ptr = is_can1 ? CAN1_0x1fe_Tx_Data : CAN2_0x1fe_Tx_Data;
    } else {
        // ID 0x209-0x20B 对应标识符 0x2FF 或 0x2FE
        if (version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            base_ptr = is_can1 ? CAN1_0x2ff_Tx_Data : CAN2_0x2ff_Tx_Data;
        else
            base_ptr = is_can1 ? CAN1_0x2fe_Tx_Data : CAN2_0x2fe_Tx_Data;
    }
    return &base_ptr[index * 2];
}

/**
 * @brief 内部函数：功率估计物理模型[cite: 1]
 */
static float power_calculate(float K0, float K1, float K2, float A, float Current, float Omega) {
    return (K0 * Current * Omega + K1 * Omega * Omega + K2 * Current * Current + A);
}

/* Class_Motor_DJI_GM6020 Implementation --------------------------------------*/

void Class_Motor_DJI_GM6020::Init(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_rx_id, Enum_Motor_DJI_Control_Method control_method, int32_t encoder_offset, Enum_Motor_DJI_GM6020_Driver_Version drive_version, Enum_Motor_DJI_Power_Limit_Status power_limit_status, float voltage_max, float current_max) {
    CAN_Manage_Object = (hcan->Instance == CAN1) ? &CAN1_Manage_Object : &CAN2_Manage_Object;
    CAN_Rx_ID = can_rx_id;
    Motor_DJI_Control_Method = control_method;
    Encoder_Offset = encoder_offset;
    Driver_Version = drive_version;
    Power_Limit_Status = power_limit_status;
    Voltage_Max = voltage_max;
    Current_Max = current_max;
    Tx_Data = allocate_tx_data(hcan, can_rx_id, drive_version);
}

void Class_Motor_DJI_GM6020::CAN_RxCpltCallback(uint8_t* Rx_Buffer) {
    Flag++;
    Data_Process();
}

void Class_Motor_DJI_GM6020::TIM_100ms_Alive_PeriodElapsedCallback() {
    Motor_DJI_Status = (Flag == Pre_Flag) ? Motor_DJI_Status_DISABLE : Motor_DJI_Status_ENABLE;
    if (Motor_DJI_Status == Motor_DJI_Status_DISABLE) {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        PID_Current.Set_Integral_Error(0.0f);
    }
    Pre_Flag = Flag;
}

void Class_Motor_DJI_GM6020::TIM_Calculate_PeriodElapsedCallback() {
    PID_Calculate();
    if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT) {
        float val = Target_Voltage + Feedforward_Voltage;
        Math_Constrain(&val, -Voltage_Max, Voltage_Max);
        Out = val * Voltage_To_Out;
    } else {
        float val = Target_Current + Feedforward_Current;
        Math_Constrain(&val, -Current_Max, Current_Max);
        Out = val * Current_To_Out;
    }
    Power_Estimate = power_calculate(Power_K_0, Power_K_1, Power_K_2, Power_A, Target_Current, Rx_Data.Now_Omega);
    Output();
}

void Class_Motor_DJI_GM6020::TIM_Power_Limit_After_Calculate_PeriodElapsedCallback() {
    if (Power_Limit_Status == Motor_DJI_Power_Limit_Status_ENABLE) Power_Limit_Control();
    // 重新计算输出以响应功率限制后的 Target
    if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT) {
        float val = Target_Voltage + Feedforward_Voltage;
        Math_Constrain(&val, -Voltage_Max, Voltage_Max);
        Out = val * Voltage_To_Out;
    } else {
        float val = Target_Current + Feedforward_Current;
        Math_Constrain(&val, -Current_Max, Current_Max);
        Out = val * Current_To_Out;
    }
    Output();
    Feedforward_Voltage = Feedforward_Current = Feedforward_Omega = 0.0f;
}

void Class_Motor_DJI_GM6020::Data_Process() {
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    auto* tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data*)CAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16(&tmp_buffer->Encoder_Reverse, &tmp_encoder);
    Math_Endian_Reverse_16(&tmp_buffer->Omega_Reverse, &tmp_omega);
    Math_Endian_Reverse_16(&tmp_buffer->Current_Reverse, &tmp_current);

    int16_t delta = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta < -Encoder_Num_Per_Round / 2) Rx_Data.Total_Round++;
    else if (delta > Encoder_Num_Per_Round / 2) Rx_Data.Total_Round--;
    
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder + Encoder_Offset;
    Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / Encoder_Num_Per_Round * 2.0f * PI;
    Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS;
    Rx_Data.Now_Current = (float)tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = (float)tmp_buffer->Temperature;
    Rx_Data.Now_Power = power_calculate(Power_K_0, Power_K_1, Power_K_2, Power_A, Rx_Data.Now_Current, Rx_Data.Now_Omega);
    Rx_Data.Pre_Encoder = tmp_encoder;
}

void Class_Motor_DJI_GM6020::PID_Calculate() {
    switch (Motor_DJI_Control_Method) {
        case Motor_DJI_Control_Method_ANGLE:
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.TIM_Calculate_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();
        case Motor_DJI_Control_Method_OMEGA:
            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();
            Target_Current = PID_Omega.Get_Out();
        case Motor_DJI_Control_Method_CURRENT:
            if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT) {
                PID_Current.Set_Target(Target_Current + Feedforward_Current);
                PID_Current.Set_Now(Rx_Data.Now_Current);
                PID_Current.TIM_Calculate_PeriodElapsedCallback();
                Target_Voltage = PID_Current.Get_Out();
            }
            break;
        default: break;
    }
}

void Class_Motor_DJI_GM6020::Power_Limit_Control() {
    if (Power_Estimate <= 0.0f || Power_Factor >= 1.0f) return;
    float a = Power_K_2, b = Power_K_0 * Rx_Data.Now_Omega;
    float c = Power_A + Power_K_1 * Rx_Data.Now_Omega * Rx_Data.Now_Omega - Power_Factor * Power_Estimate;
    float delta, h;
    delta = b * b - 4 * a * c;
    if (delta < 0.0f) Target_Current = 0.0f;
    else {
        arm_sqrt_f32(delta, &h);
        float r1 = (-b + h) / (2.0f * a), r2 = (-b - h) / (2.0f * a);
        Target_Current = (abs(r1 - Target_Current) < abs(r2 - Target_Current)) ? r1 : r2;
    }
}

void Class_Motor_DJI_GM6020::Output() {
    Tx_Data[0] = (int16_t)Out >> 8;
    Tx_Data[1] = (int16_t)Out;
}

/* Class_Motor_DJI_C610 Implementation ----------------------------------------*/

void Class_Motor_DJI_C610::Init(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_rx_id, Enum_Motor_DJI_Control_Method control_method, float gearbox_rate, float current_max) {
    CAN_Manage_Object = (hcan->Instance == CAN1) ? &CAN1_Manage_Object : &CAN2_Manage_Object;
    CAN_Rx_ID = can_rx_id;
    Motor_DJI_Control_Method = control_method;
    Gearbox_Rate = gearbox_rate;
    Current_Max = current_max;
    Tx_Data = allocate_tx_data(hcan, can_rx_id);
}

void Class_Motor_DJI_C610::CAN_RxCpltCallback(uint8_t* Rx_Buffer) {
    Flag++;
    Data_Process();
}

void Class_Motor_DJI_C610::TIM_100ms_Alive_PeriodElapsedCallback() {
    Motor_DJI_Status = (Flag == Pre_Flag) ? Motor_DJI_Status_DISABLE : Motor_DJI_Status_ENABLE;
    if (Motor_DJI_Status == Motor_DJI_Status_DISABLE) {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    Pre_Flag = Flag;
}

void Class_Motor_DJI_C610::TIM_Calculate_PeriodElapsedCallback() {
    PID_Calculate();
    float val = Target_Current + Feedforward_Current;
    Math_Constrain(&val, -Current_Max, Current_Max);
    Out = val * Current_To_Out;
    Output();
    Feedforward_Current = Feedforward_Omega = 0.0f;
}

void Class_Motor_DJI_C610::Data_Process() {
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    auto* tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data*)CAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16(&tmp_buffer->Encoder_Reverse, &tmp_encoder);
    Math_Endian_Reverse_16(&tmp_buffer->Omega_Reverse, &tmp_omega);
    Math_Endian_Reverse_16(&tmp_buffer->Current_Reverse, &tmp_current);

    int16_t delta = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta < -Encoder_Num_Per_Round / 2) Rx_Data.Total_Round++;
    else if (delta > Encoder_Num_Per_Round / 2) Rx_Data.Total_Round--;
    
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;
    Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / Encoder_Num_Per_Round / Gearbox_Rate * 2.0f * PI;
    Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Rx_Data.Now_Current = (float)tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = (float)tmp_buffer->Temperature;
    Rx_Data.Pre_Encoder = tmp_encoder;
}

void Class_Motor_DJI_C610::PID_Calculate() {
    if (Motor_DJI_Control_Method == Motor_DJI_Control_Method_ANGLE) {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();
        Target_Omega = PID_Angle.Get_Out();
    }
    if (Motor_DJI_Control_Method == Motor_DJI_Control_Method_ANGLE || Motor_DJI_Control_Method == Motor_DJI_Control_Method_OMEGA) {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        Target_Current = PID_Omega.Get_Out();
    }
}

void Class_Motor_DJI_C610::Output() {
    Tx_Data[0] = (int16_t)Out >> 8;
    Tx_Data[1] = (int16_t)Out;
}

/* Class_Motor_DJI_C620 Implementation ----------------------------------------*/

void Class_Motor_DJI_C620::Init(CAN_HandleTypeDef* hcan, Enum_Motor_DJI_ID can_rx_id, Enum_Motor_DJI_Control_Method control_method, float gearbox_rate, Enum_Motor_DJI_Power_Limit_Status power_limit_status, float current_max) {
    CAN_Manage_Object = (hcan->Instance == CAN1) ? &CAN1_Manage_Object : &CAN2_Manage_Object;
    CAN_Rx_ID = can_rx_id;
    Motor_DJI_Control_Method = control_method;
    Gearbox_Rate = gearbox_rate;
    Power_Limit_Status = power_limit_status;
    Current_Max = current_max;
    Tx_Data = allocate_tx_data(hcan, can_rx_id);
}

void Class_Motor_DJI_C620::CAN_RxCpltCallback(uint8_t* Rx_Buffer) {
    Flag++;
    Data_Process();
}

void Class_Motor_DJI_C620::TIM_100ms_Alive_PeriodElapsedCallback() {
    Motor_DJI_Status = (Flag == Pre_Flag) ? Motor_DJI_Status_DISABLE : Motor_DJI_Status_ENABLE;
    if (Motor_DJI_Status == Motor_DJI_Status_DISABLE) {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    Pre_Flag = Flag;
}

void Class_Motor_DJI_C620::TIM_Calculate_PeriodElapsedCallback() {
    PID_Calculate();
    float val = Target_Current + Feedforward_Current;
    Math_Constrain(&val, -Current_Max, Current_Max);
    Out = val * Current_To_Out;
    Power_Estimate = power_calculate(Power_K_0, Power_K_1, Power_K_2, Power_A, Target_Current, Rx_Data.Now_Omega * Gearbox_Rate);
    Output();
}

void Class_Motor_DJI_C620::TIM_Power_Limit_After_Calculate_PeriodElapsedCallback() {
    if (Power_Limit_Status == Motor_DJI_Power_Limit_Status_ENABLE) Power_Limit_Control();
    float val = Target_Current + Feedforward_Current;
    Math_Constrain(&val, -Current_Max, Current_Max);
    Out = val * Current_To_Out;
    Output();
    Feedforward_Current = Feedforward_Omega = 0.0f;
}

void Class_Motor_DJI_C620::Data_Process() {
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    auto* tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data*)CAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16(&tmp_buffer->Encoder_Reverse, &tmp_encoder);
    Math_Endian_Reverse_16(&tmp_buffer->Omega_Reverse, &tmp_omega);
    Math_Endian_Reverse_16(&tmp_buffer->Current_Reverse, &tmp_current);

    int16_t delta = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta < -Encoder_Num_Per_Round / 2) Rx_Data.Total_Round++;
    else if (delta > Encoder_Num_Per_Round / 2) Rx_Data.Total_Round--;
    
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;
    Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / Encoder_Num_Per_Round / Gearbox_Rate * 2.0f * PI;
    Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Rx_Data.Now_Current = (float)tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = (float)tmp_buffer->Temperature;
    Rx_Data.Now_Power = power_calculate(Power_K_0, Power_K_1, Power_K_2, Power_A, Rx_Data.Now_Current, Rx_Data.Now_Omega * Gearbox_Rate);
    Rx_Data.Pre_Encoder = tmp_encoder;
}

void Class_Motor_DJI_C620::PID_Calculate() {
    if (Motor_DJI_Control_Method == Motor_DJI_Control_Method_ANGLE) {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();
        Target_Omega = PID_Angle.Get_Out();
    }
    if (Motor_DJI_Control_Method == Motor_DJI_Control_Method_ANGLE || Motor_DJI_Control_Method == Motor_DJI_Control_Method_OMEGA) {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        Target_Current = PID_Omega.Get_Out();
    }
}

void Class_Motor_DJI_C620::Power_Limit_Control() {
    if (Power_Estimate <= 0.0f || Power_Factor >= 1.0f) return;
    float motor_omega = Rx_Data.Now_Omega * Gearbox_Rate;
    float a = Power_K_2, b = Power_K_0 * motor_omega;
    float c = Power_A + Power_K_1 * motor_omega * motor_omega - Power_Factor * Power_Estimate;
    float delta, h;
    delta = b * b - 4 * a * c;
    if (delta < 0.0f) Target_Current = 0.0f;
    else {
        arm_sqrt_f32(delta, &h);
        float r1 = (-b + h) / (2.0f * a), r2 = (-b - h) / (2.0f * a);
        Target_Current = (abs(r1 - Target_Current) < abs(r2 - Target_Current)) ? r1 : r2;
    }
}

void Class_Motor_DJI_C620::Output() {
    Tx_Data[0] = (int16_t)Out >> 8;
    Tx_Data[1] = (int16_t)Out;
}