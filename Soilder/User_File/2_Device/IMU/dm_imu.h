#ifndef DJI_IMU_HPP
#define DJI_IMU_HPP

#include "1_Middleware/1_Driver/CAN/drv_can.h" // 调用你提供的CAN封装

/**
 * @brief 一阶低通滤波器结构体
 */
struct Struct_Low_Pass_Filter {
    float alpha;     // 滤波系数 (0.0 ~ 1.0)，越小越平滑，但延迟越大
    float out_prev;  // 上一次的输出值

    void Init(float _alpha) {
        alpha = _alpha;
        out_prev = 0.0f;
    }

    float Update(float input) {
        float output = alpha * input + (1.0f - alpha) * out_prev;
        out_prev = output;
        return output;
    }
};

/**
 * @brief IMU 寄存器/命令定义
 */
enum Enum_IMU_Reg_ID : uint8_t {
    IMU_REG_REBOOT = 0,
    IMU_REG_ACCEL = 1,
    IMU_REG_GYRO = 2,
    IMU_REG_EULER = 3,
    IMU_REG_QUAT = 4,
    IMU_REG_SET_ZERO = 5,
    IMU_REG_ACCEL_CALI = 6,
    IMU_REG_GYRO_CALI = 7,
    IMU_REG_MAG_CALI = 8,
    IMU_REG_CHANGE_COM = 9,
    IMU_REG_SET_DELAY = 10,
    IMU_REG_CHANGE_ACTIVE = 11,
    IMU_REG_SET_BAUD = 12,
    IMU_REG_SET_CAN_ID = 13,
    IMU_REG_SET_MST_ID = 14,
    IMU_REG_SAVE_PARAM = 254,
    IMU_REG_RESTORE_SETTING = 255
};

struct Struct_IMU_Data {
    // 原始数据
    float accel[3];
    float gyro[3];
    float pitch, roll, yaw;
    float q[4];

    // 滤波后的数据
    float accel_filtered[3];
    float gyro_filtered[3];

    // 滤波器实例
    Struct_Low_Pass_Filter lpf_accel[3];
    Struct_Low_Pass_Filter lpf_gyro[3];

    uint8_t can_id;
    CAN_HandleTypeDef* hcan;

    static float uint_to_float(uint16_t x_ints, float x_min, float x_max, uint8_t bits) {
        float span = x_max - x_min;
        return ((float)x_ints) * span / ((float)((1 << bits) - 1)) + x_min;
    }
};

class Class_IMU {
public:
    Struct_IMU_Data Data;

    // 增加了 alpha 参数，默认 0.3f (建议值 0.1 ~ 0.8)
    void Init(CAN_HandleTypeDef* hcan, uint8_t can_id, float alpha_accel = 0.3f, float alpha_gyro = 0.5f);
    
    void Write_Reg(Enum_IMU_Reg_ID reg_id, uint32_t data);
	void Read_Reg(Enum_IMU_Reg_ID reg_id);
	void Save_Config();
    void Set_Active_Mode(bool active);
    void CAN_RxCpltCallback(uint8_t* rx_data);

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:
    void Send_Cmd(Enum_IMU_Reg_ID reg_id, uint8_t is_write, uint32_t data);
};

#endif