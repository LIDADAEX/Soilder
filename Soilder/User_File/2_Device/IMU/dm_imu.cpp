#include "dm_imu.h"

#define ACCEL_RANGE     235.2f
#define GYRO_RANGE      34.88f
#define EULER_P_RANGE   90.0f
#define EULER_RY_RANGE  180.0f
#define QUAT_RANGE      1.0f

void Class_IMU::Init(CAN_HandleTypeDef* hcan, uint8_t can_id, float alpha_accel, float alpha_gyro) {
    this->Data.hcan = hcan;
    this->Data.can_id = can_id;

    // 初始化滤波器
    for (int i = 0; i < 3; i++) {
        Data.lpf_accel[i].Init(alpha_accel);
        Data.lpf_gyro[i].Init(alpha_gyro);
    }
}

/**
 * @brief 发送底层协议帧
 */
void Class_IMU::Send_Cmd(Enum_IMU_Reg_ID reg_id, uint8_t is_write, uint32_t data) {
    uint8_t buf[8];
    buf[0] = 0xCC;
    buf[1] = (uint8_t)reg_id;
    buf[2] = is_write;
    buf[3] = 0xDD;
    // 使用memcpy防止字节序对齐问题
    memcpy(buf + 4, &data, 4);

    CAN_Send_Data(Data.hcan, Data.can_id, buf, 8);
}

void Class_IMU::Write_Reg(Enum_IMU_Reg_ID reg_id, uint32_t data) {
    Send_Cmd(reg_id, 1, data);
}

void Class_IMU::Read_Reg(Enum_IMU_Reg_ID reg_id) {
    Send_Cmd(reg_id, 0, 0);
}

void Class_IMU::Set_Active_Mode(bool active) {
    Write_Reg(IMU_REG_CHANGE_ACTIVE, active ? 1 : 0);
}

void Class_IMU::Save_Config() {
    Write_Reg(IMU_REG_SAVE_PARAM, 0);
}

/**
 * @brief 核心解析逻辑
 * @param rx_data CAN接收到的8字节数据指针
 */
void Class_IMU::CAN_RxCpltCallback(uint8_t* rx_data) {
    // 根据第一个字节判断数据类型
    switch (rx_data[0]) {
        case 0x01: { // Accel
            int16_t raw[3];
            raw[0] = (uint16_t)(rx_data[3] << 8 | rx_data[2]);
            raw[1] = (uint16_t)(rx_data[5] << 8 | rx_data[4]);
            raw[2] = (uint16_t)(rx_data[7] << 8 | rx_data[6]);
            for (int i = 0; i < 3; i++) 
                Data.accel[i] = Struct_IMU_Data::uint_to_float(raw[i], -ACCEL_RANGE, ACCEL_RANGE, 16);
            break;
        }
        case 0x02: { // Gyro
            int16_t raw[3];
            raw[0] = (uint16_t)(rx_data[3] << 8 | rx_data[2]);
            raw[1] = (uint16_t)(rx_data[5] << 8 | rx_data[4]);
            raw[2] = (uint16_t)(rx_data[7] << 8 | rx_data[6]);
            for (int i = 0; i < 3; i++) 
                Data.gyro[i] = Struct_IMU_Data::uint_to_float(raw[i], -GYRO_RANGE, GYRO_RANGE, 16);
            break;
        }
        case 0x03: { // Euler
            int16_t raw[3];
            raw[0] = (uint16_t)(rx_data[3] << 8 | rx_data[2]); // Pitch
            raw[1] = (uint16_t)(rx_data[5] << 8 | rx_data[4]); // Yaw
            raw[2] = (uint16_t)(rx_data[7] << 8 | rx_data[6]); // Roll
            Data.pitch = Struct_IMU_Data::uint_to_float(raw[0], -EULER_P_RANGE, EULER_P_RANGE, 16);
            Data.yaw   = Struct_IMU_Data::uint_to_float(raw[1], -EULER_RY_RANGE, EULER_RY_RANGE, 16);
            Data.roll  = Struct_IMU_Data::uint_to_float(raw[2], -EULER_RY_RANGE, EULER_RY_RANGE, 16);
            break;
        }
        case 0x04: { // Quaternion
            // 按照原逻辑的位操作解析 14bit 无符号转浮点
            uint16_t w = rx_data[1] << 6 | ((rx_data[2] & 0xF8) >> 2);
            uint16_t x = (rx_data[2] & 0x03) << 12 | (rx_data[3] << 4) | ((rx_data[4] & 0xF0) >> 4);
            uint16_t y = (rx_data[4] & 0x0F) << 10 | (rx_data[5] << 2) | (rx_data[6] & 0xC0) >> 6;
            uint16_t z = (rx_data[6] & 0x3F) << 8 | rx_data[7];
            
            Data.q[0] = Struct_IMU_Data::uint_to_float(w, -QUAT_RANGE, QUAT_RANGE, 14);
            Data.q[1] = Struct_IMU_Data::uint_to_float(x, -QUAT_RANGE, QUAT_RANGE, 14);
            Data.q[2] = Struct_IMU_Data::uint_to_float(y, -QUAT_RANGE, QUAT_RANGE, 14);
            Data.q[3] = Struct_IMU_Data::uint_to_float(z, -QUAT_RANGE, QUAT_RANGE, 14);
            break;
        }
    }
}

void Class_IMU::TIM_1ms_Calculate_PeriodElapsedCallback(){
    Read_Reg(IMU_REG_EULER);
}
