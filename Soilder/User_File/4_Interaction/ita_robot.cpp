#include "ita_robot.h"
#include "usbd_cdc_if.h"

Chassis Robot::chassis;
Class_DR16 Robot::dr16;
Class_Referee Robot::referee;
NavigationHandler Robot::navigation;

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
}

void Robot::init(){

    UART_Init(&huart1, Debug_USART1_Callback, 512);

    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);

    TIM_Init(&htim3, TIM_100ms_PeriodElapsedCallback);
	TIM_Init(&htim2, TIM_500us_PeriodElapsedCallback);

    UART_Init(&huart3, DR16_UART3_Callback, 18);
    dr16.Init(&huart3);
    LOG_INFO("遥控器初始化完成");

    motor_x_p.Init(&hcan1, Motor_DJI_ID_0x201);
    motor_x_p.PID_Omega.Init(0.5, 0.1, 0.01, 0.1, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    motor_x_m.Init(&hcan1, Motor_DJI_ID_0x203);
    motor_x_m.PID_Omega.Init(0.5, 0.1, 0.01, 0.1, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    motor_y_p.Init(&hcan1, Motor_DJI_ID_0x202);
    motor_y_p.PID_Omega.Init(0.5, 0.1, 0.01, 0.1, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    motor_y_m.Init(&hcan1, Motor_DJI_ID_0x204);
    motor_y_m.PID_Omega.Init(0.5, 0.1, 0.01, 0.1, 100, 10, 0.001, 0.1, 0.3, -2, 2, PID_D_First_ENABLE);
    LOG_INFO("底盘电机初始化完成");

    SPI_Init(&hspi1, IMU_SPI1_Callback);

    chassis.chassis_init(motor_x_p, motor_x_m, motor_y_p, motor_y_m);
    LOG_INFO("底盘初始化完成");

    UART_Init(&huart6, Referee_USART2_Callback, 512);
    referee.Init(&huart6);
    LOG_INFO("裁判系统初始化完成");

    navigation.Init(&chassis, &referee);

    HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);

}

void Robot::loop(){
    if (readFlag(flag_1ms)) {
        TIM_1ms_Calculate_PeriodElapsedCallback();
        TIM_1ms_CAN_PeriodElapsedCallback();
    }
}

void Robot::TIM_500us_PeriodElapsedCallback(){
}

void Robot::TIM_1ms_Calculate_PeriodElapsedCallback(){
    if ((dr16.Get_Status() != DR16_Status_ENABLE) && !navigation.GetAlive()) {
        chassis.Set_Control_Target(0, 0, 0, true);
    }
	chassis.m_IMU_Board.RequestAccelRead();
	chassis.m_IMU_Board.RequestGyroRead();
	
	chassis.m_IMU.TIM_1ms_Calculate_PeriodElapsedCallback();
    chassis.TIM_1ms_Calculate_PeriodElapsedCallback();
    
	navigation.GenerateNavStatus();
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

void Robot::Debug_USART1_Callback(uint8_t* Rx_Data, uint16_t Length){
    
	for (uint32_t i = 0; i < Length; i++) {
        Debug_FIFO_Push(Rx_Data[i]);
    }
}

void Robot::Referee_USART2_Callback(uint8_t* Rx_Data, uint16_t Length) {
    if(!init_finished) return;
    referee.UART_RxCpltCallback(Rx_Data, Length);
}

void Robot::IMU_SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length){
    if(((SPI1_Manage_Object.Now_GPIOx == chassis.m_IMU_Board.m_gPort) &&
        (SPI1_Manage_Object.Now_GPIO_Pin == chassis.m_IMU_Board.m_gPin)) ||
        ((SPI1_Manage_Object.Now_GPIOx == chassis.m_IMU_Board.m_aPort) &&
        (SPI1_Manage_Object.Now_GPIO_Pin == chassis.m_IMU_Board.m_aPin))){
        chassis.m_IMU_Board.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer, Length);
    }
}

void Robot::DR16_UART3_Callback(uint8_t* Rx_Data, uint16_t Length) {
    if(!init_finished) return;
    dr16.UART_RxCpltCallback(Rx_Data, Length);
    auto& data = dr16.Get_Data();
    chassis.Set_Control_Target(data.Left_X, data.Left_Y, data.Side_Wheel, true);
} 

void Robot::TIM_100ms_PeriodElapsedCallback() {
    if(!init_finished) return;
    dr16.TIM_Alive_PeriodElapsedCallback();
	chassis.m_IMU_Board.TIM_100ms_Callback();
    navigation.TIM_100ms_Callback();
	
}

