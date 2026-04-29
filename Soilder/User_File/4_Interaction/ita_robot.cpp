#include "ita_robot.h"
#include "usbd_cdc_if.h"

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

enum class GimbalMode{
    none,
    controlNoFire,
    controlAndFile
};

#pragma pack(push, 1) // 强制 1 字节对齐，防止编译器乱加填充
struct Struct_Robot_Main_Packet {
	struct{
		uint8_t header[2];   // 'S', 'P'
		float q[4] = {0};          // 四元数
		float yaw = 0;
		//float yaw_vel = 0;
		float pitch = 0;
		//float pitch_vel = 0;
		float shootSpeed;
		uint16_t ammo_remain; // 弹丸剩余通常是 uint16_t
		uint16_t crc16;
	}struct_Gimbal_Send_Packet;

	struct{
		uint8_t header;   
		uint8_t Stage_Enum;
		uint16_t Remaining_Time;
		uint16_t Current_hp;
		uint8_t middle_buff_status;
		// uint8_t crc16;
		uint8_t tail;
	}struct_navPacket ;
};
#pragma pack(pop)

static uint8_t flag = 0;
static uint8_t past_flag = 0;
enum class ControlState{
    enable,
    disable
}controlState;


void Robot::TIM_1ms_Calculate_PeriodElapsedCallback(){
    if ((dr16.Get_Status() != DR16_Status_ENABLE) && (controlState != ControlState::enable)) {
        chassis.Set_Control_Target(0, 0, 0, true);
    }
    chassis.TIM_1ms_Calculate_PeriodElapsedCallback();
    chassis.m_IMU.TIM_1ms_Calculate_PeriodElapsedCallback();

    Struct_Robot_Main_Packet packet;
    packet.struct_navPacket.header = '@';
    packet.struct_navPacket.Stage_Enum = Robot::referee.Game_Status.Stage_Enum;
    packet.struct_navPacket.Remaining_Time = Robot::referee.Game_Status.Remaining_Time;
    packet.struct_navPacket.Current_hp = Robot::referee.Robot_Status.HP;
    packet.struct_navPacket.middle_buff_status = Robot::referee.Robot_Buff.Defend_Buff_Percent;
    packet.struct_navPacket.tail = '#';

    //spacket.struct_navPacket.crc16 = Class_Referee::Get_CRC16((uint8_t*)&packet.struct_navPacket, 8, 0xffff);

    static GimbalMode mode = GimbalMode::none;
    
    packet.struct_Gimbal_Send_Packet.header[0] = 'S';
    packet.struct_Gimbal_Send_Packet.header[1] = 'P';
    
    packet.struct_Gimbal_Send_Packet.shootSpeed = Robot::referee.Get_Shoot_Initial_Speed();
    packet.struct_Gimbal_Send_Packet.ammo_remain = Robot::referee.Get_Ammo_17mm_1_Remain();
    packet.struct_Gimbal_Send_Packet.yaw = Robot::chassis.m_IMU.Data.yaw;
    packet.struct_Gimbal_Send_Packet.pitch = Robot::chassis.m_IMU.Data.pitch;

    packet.struct_Gimbal_Send_Packet.crc16 = Class_Referee::Get_CRC16((uint8_t*)&packet.struct_Gimbal_Send_Packet, 6 + 7 * sizeof(float), 0xffff);

    CDC_Transmit_FS((uint8_t*)&packet, 14 + 7 * sizeof(float));
}

void cmd_0x6A(uint8_t* cmd){
    Robot::chassis.Set_Control_Target(*(float*)cmd,*(float*) (cmd + sizeof(float)), 0, true);
}

void Robot::Controlcmd_DataProcess(uint8_t* Rx_Data, uint16_t Length){
    uint8_t startPos = 0;
    flag ++;
    while(startPos != Length){
        switch(Rx_Data[startPos]){
            case 0x6A:{
                cmd_0x6A(Rx_Data + startPos + 1);
                startPos += 10;
            }
            break;
        }
    }
    
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

    if(past_flag != flag){
        past_flag = flag;
        controlState = ControlState::enable;
    }
    else{
        controlState = ControlState::disable;
    }
}

