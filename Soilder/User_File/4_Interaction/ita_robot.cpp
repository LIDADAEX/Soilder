#include "ita_robot.h"

Chassis Robot::chassis;
Class_DR16 Robot::dr16;
Class_Referee Robot::referee;

bool flag_1ms = false;

extern bool init_finished;

bool readFlag(bool& flag) {
    bool flagTemp = flag;
    flag = false;
    return flagTemp;
}

void HAL_SYSTICK_Callback(void) {
    flag_1ms = true;
}


void Robot::init(){

    UART_Init(&huart6, Debug_USART6_Callback, 512);

    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);

    TIM_Init(&htim3, TIM_100ms_PeriodElapsedCallback);

    UART_Init(&huart3, DR16_UART3_Callback, 18);
    dr16.Init(&huart3);
    LOG_INFO("遥控器初始化完成");

    motor_x_p.Init(&hcan1, Motor_DJI_ID_0x201);
    motor_x_p.PID_Omega.Init(2.0, 0.1, 0.01, 1.0, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    motor_x_m.Init(&hcan1, Motor_DJI_ID_0x203);
    motor_x_m.PID_Omega.Init(2.0, 0.1, 0.01, 1.0, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    motor_y_p.Init(&hcan1, Motor_DJI_ID_0x202);
    motor_y_p.PID_Omega.Init(2.0, 0.1, 0.01, 1.0, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    motor_y_m.Init(&hcan1, Motor_DJI_ID_0x204);
    motor_y_m.PID_Omega.Init(2.0, 0.1, 0.01, 1.0, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    LOG_INFO("底盘电机初始化完成");

    chassis.chassis_init(motor_x_p, motor_x_m, motor_y_p, motor_y_m);
    LOG_INFO("底盘初始化完成");

    UART_Init(&huart4, Referee_USART4_Callback, 512);
    referee.Init(&huart4);
    LOG_INFO("裁判系统初始化完成");

    HAL_TIM_Base_Start_IT(&htim3);

}

void Robot::loop(){
    if (readFlag(flag_1ms)) {
        TIM_1ms_Calculate_PeriodElapsedCallback();
        TIM_1ms_CAN_PeriodElapsedCallback();
    }
}

void Robot::TIM_1ms_Calculate_PeriodElapsedCallback(){
    if (dr16.Get_Status() != DR16_Status_ENABLE) {
        chassis.Set_Control_Target(0, 0, 0, true);
    }
    chassis.TIM_1ms_Calculate_PeriodElapsedCallback();
    chassis.m_IMU.TIM_1ms_Calculate_PeriodElapsedCallback();
}

void Robot::Device_CAN1_Callback(Struct_CAN_Rx_Buffer* CAN_RxMessage) {
    if(!init_finished) return;
    switch (CAN_RxMessage->Header.StdId) {
        case (0x201): {
            motor_x_p.CAN_RxCpltCallback(CAN_RxMessage->Data);
        } break;
        case (0x203): {
            motor_x_m.CAN_RxCpltCallback(CAN_RxMessage->Data);
        } break;
        case (0x202): {
            motor_y_p.CAN_RxCpltCallback(CAN_RxMessage->Data);
        } break;
        case (0x204): {
            motor_y_m.CAN_RxCpltCallback(CAN_RxMessage->Data);
        } break;
    }
}

void Robot::Device_CAN2_Callback(Struct_CAN_Rx_Buffer* CAN_RxMessage) {
    if(!init_finished) return;
    switch (CAN_RxMessage->Header.StdId) {
        case (0x55): {
            chassis.m_IMU.CAN_RxCpltCallback(CAN_RxMessage->Data);
        } break;
    }
}

extern void Debug_FIFO_Push(uint8_t data);

void Robot::Debug_USART6_Callback(uint8_t* Rx_Data, uint16_t Length){
    for (uint32_t i = 0; i < Length; i++) {
        Debug_FIFO_Push(Rx_Data[i]);
    }
}

void Robot::Referee_USART4_Callback(uint8_t* Rx_Data, uint16_t Length) {
    if(!init_finished) return;
    referee.UART_RxCpltCallback(Rx_Data, Length);
}

void Robot::DR16_UART3_Callback(uint8_t* Rx_Data, uint16_t Length) {
    if(!init_finished) return;
    dr16.UART_RxCpltCallback(Rx_Data, Length);
    chassis.Set_Control_Target(dr16.Get_Left_X(), dr16.Get_Left_Y(), dr16.Get_Yaw(), true);
} 

void Robot::TIM_100ms_PeriodElapsedCallback() {
    if(!init_finished) return;
    dr16.TIM_100ms_Alive_PeriodElapsedCallback();
}

