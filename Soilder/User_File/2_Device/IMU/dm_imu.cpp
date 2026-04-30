#include "dm_imu.h"

/* Private Constants ---------------------------------------------------------*/

// 根据达妙协议手册定义的量程
static const float ACCEL_RANGE   = 235.2f;  // 24g * 9.8
static const float GYRO_RANGE    = 34.88f;  // 2000deg/s
static const float EULER_P_RANGE = 90.0f;
static const float EULER_RY_RANGE = 180.0f;
static const float QUAT_RANGE    = 1.0f;

/* Function Prototypes -------------------------------------------------------*/

void Class_IMU::Init(CAN_HandleTypeDef* hcan, uint16_t CAN_ID, float Alpha_Accel, float Alpha_Gyro) {
    m_hcan = hcan;
    m_CAN_ID = CAN_ID;

    for (int i = 0; i < 3; i++) {
        m_LPF_Accel[i].Init(Alpha_Accel);
        m_LPF_Gyro[i].Init(Alpha_Gyro);
    }
}

void Class_IMU::Send_Cmd(Enum_IMU_Reg_ID Reg_ID, uint8_t Is_Write, uint32_t Data) {
    uint8_t buf[8];
    buf[0] = 0xCC;
    buf[1] = (uint8_t)Reg_ID;
    buf[2] = Is_Write;
    buf[3] = 0xDD;
    // 拷贝数据，保持小端序
    memcpy(buf + 4, &Data, 4);

    CAN_Send_Data(m_hcan, m_CAN_ID, buf, 8);
}

void Class_IMU::Write_Reg(Enum_IMU_Reg_ID Reg_ID, uint32_t Data) {
    Send_Cmd(Reg_ID, 1, Data);
}

void Class_IMU::Read_Reg(Enum_IMU_Reg_ID Reg_ID) {
    Send_Cmd(Reg_ID, 0, 0);
}

void Class_IMU::Set_Active_Mode(bool Active) {
    Write_Reg(IMU_REG_CHANGE_ACTIVE, Active ? 1 : 0);
}

void Class_IMU::Save_Config() {
    Write_Reg(IMU_REG_SAVE_PARAM, 0);
}

void Class_IMU::CAN_RxCpltCallback(uint8_t* Rx_Data) {
    if (Rx_Data == nullptr) return;

    switch (Rx_Data[0]) {
        case 0x01: { // 加速度解析
            for (int i = 0; i < 3; i++) {
                uint16_t raw = (uint16_t)(Rx_Data[i * 2 + 3] << 8 | Rx_Data[i * 2 + 2]);
                m_Data.Accel[i] = Math_Uint_To_Float(raw, -ACCEL_RANGE, ACCEL_RANGE, 16);
                m_Data.Accel_Filtered[i] = m_LPF_Accel[i].Update(m_Data.Accel[i]);
            }
            break;
        }
        case 0x02: { // 角速度解析
            for (int i = 0; i < 3; i++) {
                uint16_t raw = (uint16_t)(Rx_Data[i * 2 + 3] << 8 | Rx_Data[i * 2 + 2]);
                m_Data.Gyro[i] = Math_Uint_To_Float(raw, -GYRO_RANGE, GYRO_RANGE, 16);
                m_Data.Gyro_Filtered[i] = m_LPF_Gyro[i].Update(m_Data.Gyro[i]);
            }
            break;
        }
        case 0x03: { // 欧拉角解析
            uint16_t raw_p = (uint16_t)(Rx_Data[3] << 8 | Rx_Data[2]);
            uint16_t raw_y = (uint16_t)(Rx_Data[5] << 8 | Rx_Data[4]);
            uint16_t raw_r = (uint16_t)(Rx_Data[7] << 8 | Rx_Data[6]);

            m_Data.Pitch = Math_Uint_To_Float(raw_p, -EULER_P_RANGE, EULER_P_RANGE, 16);
            m_Data.Yaw   = Math_Uint_To_Float(raw_y, -EULER_RY_RANGE, EULER_RY_RANGE, 16);
            m_Data.Roll  = Math_Uint_To_Float(raw_r, -EULER_RY_RANGE, EULER_RY_RANGE, 16);
            break;
        }
        case 0x04: { // 四元数解析 (特殊 14bit 协议)
            uint16_t w = Rx_Data[1] << 6 | ((Rx_Data[2] & 0xF8) >> 2);
            uint16_t x = (Rx_Data[2] & 0x03) << 12 | (Rx_Data[3] << 4) | ((Rx_Data[4] & 0xF0) >> 4);
            uint16_t y = (Rx_Data[4] & 0x0F) << 10 | (Rx_Data[5] << 2) | (Rx_Data[6] & 0xC0) >> 6;
            uint16_t z = (Rx_Data[6] & 0x3F) << 8 | Rx_Data[7];

            m_Data.Quat[0] = Math_Uint_To_Float(w, -QUAT_RANGE, QUAT_RANGE, 14);
            m_Data.Quat[1] = Math_Uint_To_Float(x, -QUAT_RANGE, QUAT_RANGE, 14);
            m_Data.Quat[2] = Math_Uint_To_Float(y, -QUAT_RANGE, QUAT_RANGE, 14);
            m_Data.Quat[3] = Math_Uint_To_Float(z, -QUAT_RANGE, QUAT_RANGE, 14);
            break;
        }
    }
}

void Class_IMU::TIM_1ms_Calculate_PeriodElapsedCallback() {
    // 轮询请求欧拉角数据，也可以根据需要请求 ACCEL 或 GYRO
    Read_Reg(IMU_REG_EULER);
}