/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如底盘陀螺仪对象, 底盘类要调用它做小陀螺, 云台要调用它做方位感知, 因此底盘陀螺仪是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "stm32f407xx.h"
#include "1_Middleware/3_Debug/debug_log.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "1_Middleware/3_Debug/debug_cmd_interface.h"
#include "3_Chariot/1_Module/Chassis/crt_chassis.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "2_Device/DR16/dvc_dr16.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;
uint32_t flag = 0;

Class_Motor_DJI_C620 motor_x_p;
Class_Motor_DJI_C620 motor_x_m;
Class_Motor_DJI_C620 motor_y_p;
Class_Motor_DJI_C620 motor_y_m;
Chassis chassis;

Class_DR16 dr16;

bool flag_chassisTIM_1ms = false;
bool flag_dr16TIM_1ms = false;

bool readFlag(bool& flag){
    bool flagTemp = flag;
    flag = false;
    return flagTemp;
}

void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        motor_x_p.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        motor_x_m.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        motor_y_p.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        motor_y_m.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}

void Device_UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length){
    dr16.UART_RxCpltCallback(Rx_Data, Length);
}

void HAL_SYSTICK_Callback(void)
{
    flag_chassisTIM_1ms = true;
    flag_dr16TIM_1ms = true;
}

void TIM_100ms_PeriodElapsedCallback(){
    chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    dr16.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    CAN_Init(&hcan1, Device_CAN1_Callback);
    TIM_Init(&htim3, TIM_100ms_PeriodElapsedCallback);

    motor_x_p.Init(&hcan1, Motor_DJI_ID_0x201);
    motor_x_m.Init(&hcan1, Motor_DJI_ID_0x202);
    motor_y_p.Init(&hcan1, Motor_DJI_ID_0x203);
    motor_y_m.Init(&hcan1, Motor_DJI_ID_0x204);
    LOG_INFO("底盘电机初始化完成");

    // UART_Init(&huart2, Device_UART_RxCpltCallback, UART_BUFFER_SIZE);
    // dr16.Init(&huart2);
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
	Debug_Cmd_Poll_Callback();

    if(readFlag(flag_chassisTIM_1ms)){
        if(dr16.Get_Status() == DR16_Status_ENABLE){
            chassis.Set_Control_Target(dr16.Get_Left_X(), dr16.Get_Left_Y(), dr16.Get_Right_X(), true);
            chassis.TIM_1ms_Calculate_PeriodElapsedCallback();
        }
    }
        
    if(readFlag(flag_dr16TIM_1ms))
        dr16.TIM_1ms_Calculate_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
