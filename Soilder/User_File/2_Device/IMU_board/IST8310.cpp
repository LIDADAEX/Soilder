#include "IST8310.h"
#include <stdio.h>

IST8310::Data IST8310::sm_data = {0};

bool IST8310::Init(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* drdyPort, uint16_t drdyPin, GPIO_TypeDef* nrstPort, uint16_t nrstPin) {
    m_hi2c = hi2c;
    m_drdyPort = drdyPort;
    m_drdyPin = drdyPin;
	m_nrstPort = nrstPort;
    m_nrstPin = nrstPin;
	
	HAL_GPIO_WritePin(m_nrstPort, m_nrstPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(m_nrstPort, m_nrstPin, GPIO_PIN_SET);
	HAL_Delay(50);
	
    // 1. 检查 WHO_AM_I 寄存器 (阻塞读取)
    uint8_t who_am_i = ReadReg(0x00);
    if (who_am_i != 0x10) {
        return false;
    }

    // 2. 硬件配置 (阻塞写入)
	WriteReg(0x0B, 0x0C); // 开启中断输出引脚 + 电平保持模式（假设 bit1=1）
    HAL_Delay(10);
    WriteReg(0x41, 0x09); // 平均采样配置
    WriteReg(0x42, 0xC0); // 必须的内部寄存器配置
    HAL_Delay(10);

    // 3. 启动第一次测量，踢皮球的第一脚，后续将由中断闭环接管
	WriteReg(0x0B, 0x01); // 启动测量，DRDY 将在数据准备好后变为高并保持，直到你读取数据后自动清零

	isInitialed = true;
    return true;
}

void IST8310::WriteReg(uint8_t reg, uint8_t val) {
    uint8_t pData[2] = {reg, val};
    HAL_I2C_Master_Transmit(m_hi2c, IST8310_IIC_ADDRESS, pData, 2, 10);
}

uint8_t IST8310::ReadReg(uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(m_hi2c, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

void IST8310::EXTI_Callback(uint16_t GPIO_Pin) {
    // 判断是否是磁力计的 DRDY 引脚触发了中断
    if (this->isInitialed && GPIO_Pin == m_drdyPin) {
        
        // 此时数据已就绪，由于不能在中断里阻塞等待，我们通过 DMA 发送要读取的寄存器首地址 0x03
        if (m_state == I2C_State::IDLE) {
            m_state = I2C_State::SET_READ_ADDR;
            uint8_t reg_addr = 0x03;
            // 调用你的 drv_iic 发送函数
            while(I2C_Send_Data(m_hi2c, IST8310_IIC_ADDRESS, &reg_addr, 1) != HAL_OK);
        }
    }
}

void IST8310::I2C_TxRxCpltCallback(uint16_t DevAddress, uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length) {
    if (DevAddress != IST8310_IIC_ADDRESS) return;

    // 1. 发送读取地址完成 (根据你的驱动，发送完成时 Rx_Length 为 0[cite: 1])
    if (m_state == I2C_State::SET_READ_ADDR && Rx_Length == 0) {
        m_state = I2C_State::READING_DATA;
        // 紧接着请求 DMA 读取 6 个字节的数据[cite: 1]
        while(I2C_Reccive_Data(m_hi2c, IST8310_IIC_ADDRESS, 6) != HAL_OK);
    }
    // 2. 数据读取完成
    else if (m_state == I2C_State::READING_DATA && Rx_Length == 6) {
        // 解析数据
        const float scale = 0.3f; 
        int16_t raw_x = (int16_t)((Rx_Buffer[1] << 8) | Rx_Buffer[0]);
        int16_t raw_y = (int16_t)((Rx_Buffer[3] << 8) | Rx_Buffer[2]);
        int16_t raw_z = (int16_t)((Rx_Buffer[5] << 8) | Rx_Buffer[4]);

        sm_data.mx = raw_x * scale;
        sm_data.my = raw_y * scale;
        sm_data.mz = raw_z * scale;
		
		//printf("{IST}%.5f,%.5f,%.5f\n", sm_data.mx, sm_data.my, sm_data.mz);

        // 【算法更新点】此处可以将数据推给你的 IMU 解算器

        // 数据读完后，立刻触发下一次单次测量以实现连续循环
        m_state = I2C_State::TRIGGERING;
        uint8_t trigger_cmd[2] = {0x0A, 0x01};
        // 调用发送函数发送命令[cite: 1]
        while(I2C_Send_Data(m_hi2c, IST8310_IIC_ADDRESS, trigger_cmd, 2) != HAL_OK);
    }
    // 3. 触发命令发送完成
    else if (m_state == I2C_State::TRIGGERING && Rx_Length == 0) {
        // 彻底完成一次循环，回归空闲状态，等待下一次 DRDY 中断降临
        m_state = I2C_State::IDLE;
    }
}