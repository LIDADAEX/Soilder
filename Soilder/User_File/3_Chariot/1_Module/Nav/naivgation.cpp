#include "naivgation.h"

void NavigationHandler::CMDProcess(uint8_t* buf, uint32_t len) {
        // 1. 长度基础检查
    if (len < sizeof(CmdVelPacket)) return;

    // 2. 帧头校验 (0x6A)
    if (buf[0] != 0x6A) return;

    // 3. CRC16 校验计算
    // 关键点：计算长度 = 结构体总长度 - 校验和字段长度 (2字节)
    const uint32_t data_len_to_verify = sizeof(CmdVelPacket) - sizeof(uint16_t);
    uint16_t calc_crc = Class_Referee::Get_CRC16(buf, data_len_to_verify, 0xFFFF);
    
    // 提取收到的校验和（注意字节序，这里假设为小端）
    uint16_t recv_crc;
    memcpy(&recv_crc, buf + data_len_to_verify, sizeof(uint16_t));

    if (calc_crc != recv_crc) {
        // 校验失败，报错或直接返回
        return; 
    }

    // 4. 解析数据并映射到静态对象 Robot::chassis
    CmdVelPacket* pkg = reinterpret_cast<CmdVelPacket*>(buf);

    m_ptr_chassis->Set_Target_VX(pkg->linear_x);
    m_ptr_chassis->Set_Target_VY(pkg->linear_y);
    
    // 映射位域信息
    m_ptr_chassis->Set_World_Frame_Status(pkg->config.bits.chassis_mode == 1);
        
}

void NavigationHandler::GenerateNavStatus() {
    NavPacket pkg;
    pkg.stage_enum = m_ptr_referee->Get_Game_Type();
    pkg.stage_remain_time = m_ptr_referee->Get_Stage_Remain_Time();
    pkg.current_HP = m_ptr_referee->Get_Current_HP();
    pkg.middle_buff_status = 0;
    CDC_Transmit_FS((uint8_t*)&pkg, sizeof(pkg));
}

void NavigationHandler::TIM_100ms_Callback(){
    if(m_flag == m_lastFlag){
        m_isAlive = false;
    }else
    {
        m_isAlive = true;
        m_lastFlag = m_flag;
    }
}