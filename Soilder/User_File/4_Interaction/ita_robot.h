#ifndef ITA_ROBOT_H
#define ITA_ROBOT_H

#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "1_Middleware/3_Debug/debug_cmd_interface.h"
#include "1_Middleware/3_Debug/debug_log.h"

#include "2_Device/DR16/dvc_dr16.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "3_Chariot/1_Module/Chassis/crt_chassis.h"

class Robot{

public:
    static Chassis chassis;
    static Class_DR16 dr16;

    static void init();
    static void loop();
private:

    static void TIM_1ms_Calculate_PeriodElapsedCallback();
    static void TIM_100ms_PeriodElapsedCallback();

    static void Device_CAN1_Callback(Struct_CAN_Rx_Buffer* CAN_RxMessage);
    static void Device_CAN2_Callback(Struct_CAN_Rx_Buffer* CAN_RxMessage);
    static void DR16_UART3_Callback(uint8_t* Rx_Data, uint16_t Length);

};

#endif
