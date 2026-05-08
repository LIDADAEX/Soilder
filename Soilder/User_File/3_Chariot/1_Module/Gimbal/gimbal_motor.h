#ifndef __GIMBAL_MOTOR_H_
#define __GIMBAL_MOTOR_H_

#include "dm_imu.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"

/**
 * @brief 云台轴类型
 *
 */
typedef enum { The_Pitch = 0, The_Yaw } Gimbal_Axis_Type;

/**
 * @brief 云台电机控制模式（应用层）
 *
 */
enum Enum_Gimbal_Motor_Control_Mode {
    Gimbal_Motor_Control_Mode_ANGLE = 0,  // 角度模式
    Gimbal_Motor_Control_Mode_OMEGA,       // 速度模式
};

/**
 * @brief 这是一坨针对云台关节电机的达妙电机类复写
 *
 */
class Class_4310_Joint_With_IMU : public Class_Motor_DM_Normal {
public:
  void Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID,
            Gimbal_Axis_Type __axis_type,
            Enum_Motor_DM_Control_Method __Motor_DM_Control_Method =
                Motor_DM_Control_Method_NORMAL_MIT,
            float __Angle_Max = 12.5f, float __Omega_Max = 30.0f,
            float __Torque_Max = 10.0f, float __Current_Max = 10.261194f);

  void Bind_IMU(Class_DM_IMU *__IMU);

  Gimbal_Axis_Type axis_type;

  Class_PID PID_Angle;

  Class_PID PID_Omega;

  Class_DM_IMU *Motor_Absolute_Angle;

  Class_Filter_Kalman Filter_Omega_DM_4310;

  virtual inline float Get_Now_Angle();

  inline float Get_Real_Angle();

  inline float Get_Target_Angle();

  virtual inline float Get_Now_Omega();

  inline float Get_Real_Omega();

  inline void Set_Target_Angle(float __Target_Angle);

  inline void Set_Target_Omega(float __Target_Omega);

  inline void Set_Control_Mode(Enum_Gimbal_Motor_Control_Mode __Control_Mode);

  inline Enum_Gimbal_Motor_Control_Mode Get_Control_Mode();

  inline void Set_Control_FeedForward_Omega(float feedforward);
  inline void Set_Control_FeedForward_Current(float feedforward);
  inline void Set_Control_FeedForward_Angle(float feedforward);

  virtual void RTOS_100ms_Alive_Callback();

  void RTOS_1ms_Calculate_Callback();
    // 电机反馈的角速度
  float after_filter_Omega = 0.0f;

private:

  float Target_Angle = 0.0f;

  float Target_Omega = 0.0f;

  Enum_Gimbal_Motor_Control_Mode Control_Mode = Gimbal_Motor_Control_Mode_ANGLE;

  float Control_FeedForward_Angle = 0.0f;

  float Control_FeedForward_Omega = 0.0f;

  float Control_FeedForward_Current = 0.0f;

  void PID_Calculate();

  virtual void Output();
};

class Class_GM6020_Joint_With_IMU : public Class_Motor_DJI_GM6020 {
public:
  void Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID,
            Gimbal_Axis_Type __axis_type = The_Pitch,
            Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method =
                Motor_DJI_Control_Method_ANGLE,
            int32_t __Encoder_Offset = 0,
            Enum_Motor_DJI_GM6020_Driver_Version __Drive_Version =
                Motor_DJI_GM6020_Driver_Version_DEFAULT,
            Enum_Motor_DJI_Power_Limit_Status __Power_Limit_Status =
                Motor_DJI_Power_Limit_Status_DISABLE,
            float __Voltage_Max = 25000.0f, float __Current_Max = 16384.0f);

  void Bind_IMU(Class_DM_IMU *__IMU);

  Gimbal_Axis_Type axis_type;

  Class_DM_IMU *Motor_Absolute_Angle;

  Class_Filter_Fourier<10> Filter_Omega_GM6020;

  virtual inline float Get_Now_Angle();

  inline float Get_Real_Angle();

  virtual inline float Get_Now_Omega();

  inline float Get_Real_Omega();

  virtual void RTOS_Calculate_Callback();

private:
  float after_filter_Omega = 0.0f;

  // 内部函数
  virtual void PID_Calculate();

  virtual void Power_Limit_Control();
};

class Class_Filter_GM6020_Joint : public Class_Motor_DJI_GM6020 {
public:
  void Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, float *big_yaw_omega,
            Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method =
                Motor_DJI_Control_Method_ANGLE,
            int32_t __Encoder_Offset = 0,
            Enum_Motor_DJI_GM6020_Driver_Version __Drive_Version =
                Motor_DJI_GM6020_Driver_Version_DEFAULT,
            Enum_Motor_DJI_Power_Limit_Status __Power_Limit_Status =
                Motor_DJI_Power_Limit_Status_DISABLE,
            float __Voltage_Max = 25000.0f, float __Current_Max = 16384.0f);         

  virtual inline float Get_Now_Omega();

  inline float Get_Real_Omega();

  virtual void RTOS_Calculate_Callback();


private:
  float after_filter_Omega = 0.0f;

  float *Big_Yaw_Omega = nullptr;

  Class_Filter_Kalman Filter_Omega_Kalman;
  // 内部函数
  virtual void PID_Calculate();

  virtual void Power_Limit_Control();
};

/**
 * @brief 外置目标角度值
 *
 * @param __Target_Angle
 */
inline void Class_4310_Joint_With_IMU::Set_Target_Angle(float __Target_Angle) {
  Target_Angle = __Target_Angle;
}

/**
 * @brief 设置目标角速度（速度模式）
 *
 * @param __Target_Omega 目标角速度 rad/s
 */
inline void Class_4310_Joint_With_IMU::Set_Target_Omega(float __Target_Omega) {
  Target_Omega = __Target_Omega;
}

/**
 * @brief 设置控制模式（角度模式/速度模式）
 *
 * @param __Control_Mode 控制模式
 */
inline void Class_4310_Joint_With_IMU::Set_Control_Mode(Enum_Gimbal_Motor_Control_Mode __Control_Mode) {
  Control_Mode = __Control_Mode;
}

/**
 * @brief 获取控制模式
 *
 * @return Enum_Gimbal_Motor_Control_Mode 控制模式
 */
inline Enum_Gimbal_Motor_Control_Mode Class_4310_Joint_With_IMU::Get_Control_Mode() {
  return Control_Mode;
}


inline void Class_4310_Joint_With_IMU::Set_Control_FeedForward_Angle(float feedforward) {
  Control_FeedForward_Angle = feedforward;
}
/**
 * @brief 设置前馈速度，用于小陀螺云台自稳
 *
 * @param feedforward
 */
inline void
Class_4310_Joint_With_IMU::Set_Control_FeedForward_Omega(float feedforward) {
  Control_FeedForward_Omega = feedforward;
}

/**
 * @brief 设置前馈电流，用于小陀螺云台自稳
 *
 * @param feedforward
 */
inline void
Class_4310_Joint_With_IMU::Set_Control_FeedForward_Current(float feedforward) {
  Control_FeedForward_Current = feedforward;
}



inline float Class_4310_Joint_With_IMU::Get_Target_Angle() {
  return (Target_Angle);
}

/**
 * @brief 复写
 *
 * @return float
 */
inline float Class_4310_Joint_With_IMU::Get_Now_Angle() {
  if (axis_type == The_Yaw) {
    return (Motor_Absolute_Angle->Total_Angle_Yaw * DEG_TO_RAD);
  } else {
    return (Motor_Absolute_Angle->Get_Angle_Pitch() * DEG_TO_RAD);
  }
}

/**
 *
 */
inline float Class_4310_Joint_With_IMU::Get_Real_Angle() {
  return (Rx_Data.Now_Angle);
}

/**
 * @brief 滤波后的角速度
 *
 * @return float
 */
inline float Class_4310_Joint_With_IMU::Get_Now_Omega() {
  return after_filter_Omega;
}

inline float Class_4310_Joint_With_IMU::Get_Real_Omega() {
   return (Rx_Data.Now_Omega);
}

inline float Class_Filter_GM6020_Joint::Get_Now_Omega() {
  return after_filter_Omega;
}

inline float Class_Filter_GM6020_Joint::Get_Real_Omega() {
  return (Rx_Data.Now_Omega);
}

inline float Class_GM6020_Joint_With_IMU::Get_Now_Omega() {
  return after_filter_Omega;
}

inline float Class_GM6020_Joint_With_IMU::Get_Now_Angle() {
  if (axis_type == The_Yaw) {
    return (Motor_Absolute_Angle->Get_Angle_Yaw());
  } else {
    return (Motor_Absolute_Angle->Get_Angle_Pitch());
  }
}

inline float Class_GM6020_Joint_With_IMU::Get_Real_Omega() {
  return (Rx_Data.Now_Omega);
}

inline float Class_GM6020_Joint_With_IMU::Get_Real_Angle() {
  return (Rx_Data.Now_Angle);
}

#endif
