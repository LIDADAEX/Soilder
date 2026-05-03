#pragma once

#include "3_Chariot/1_Module/Chassis/crt_chassis.h"
#include "2_Device/Referee/dvc_referee.h"
#include "usbd_cdc_if.h"

// 保持 packed 属性确保内存对齐
struct NavPacket {
    uint8_t header = '@';
    uint8_t stage_enum;
    uint16_t stage_remain_time;
    uint16_t current_HP;
    uint8_t middle_buff_status;
    uint8_t end_of_frame = '#';
} __attribute__((packed));

struct CmdVelPacket
{
    uint8_t header = 0x6A;
    float linear_x;
    float linear_y;
    union
    {
        uint8_t all = 0;
        struct 
        {
            uint8_t chassis_mode : 1;  // 1->世界坐标系, 0->机器人坐标系
            uint8_t scan_status : 1;
            uint8_t reserved : 6;
        } bits; // 命名为bits以便访问
    } config;
    uint16_t checksum = 0;
} __attribute__((packed));

class NavigationHandler {
public:
    /**
     * @brief 处理收到的控制指令包
     * @param buf 原始数据缓冲区
     * @param len 数据长度
     */

    void Init(Chassis* ptr_chassis, Class_Referee* ptr_referee){
        m_ptr_chassis = ptr_chassis;
        m_ptr_referee = ptr_referee;
    }
    

    void CMDProcess(uint8_t* buf, uint32_t len);

    /**
     * @brief 发送状态回传包 (NavPacket)
     * @return 填充好的 NavPacket
     */
    void GenerateNavStatus();

    void TIM_100ms_Callback();

    bool GetAlive(){return m_isAlive;};

private:

    Chassis* m_ptr_chassis;
    Class_Referee* m_ptr_referee;

    bool m_isAlive = false;
    uint32_t m_flag = 0;
    uint32_t m_lastFlag = 0;
};