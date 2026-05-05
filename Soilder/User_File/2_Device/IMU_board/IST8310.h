#pragma once

#include "1_Middleware/1_Driver/IIC/drv_iic.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)

class IST8310 {
public:
    struct Data {
        float mx, my, mz;
    };

    // I2C 异步通信状态机
    enum class I2C_State {
        IDLE,           // 空闲
        SET_READ_ADDR,  // 正在发送读取地址
        READING_DATA,   // 正在读取数据
        TRIGGERING      // 正在触发下一次测量
    };

    I2C_HandleTypeDef* m_hi2c;
    GPIO_TypeDef*      m_drdyPort;
    uint16_t           m_drdyPin;
	GPIO_TypeDef*      m_nrstPort;
    uint16_t           m_nrstPin ;
    // 增加了 DRDY 外部中断引脚的传入
    bool Init(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* drdyPort, uint16_t drdyPin, GPIO_TypeDef* nrstPort, uint16_t nrstPin);
    
    // 外部中断 EXTI 回调，放在 HAL_GPIO_EXTI_Callback 中调用
    void EXTI_Callback(uint16_t GPIO_Pin);

    // 适配你驱动格式的 DMA 回调函数
    void I2C_TxRxCpltCallback(uint16_t DevAddress, uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);
    
    Data GetData() { return sm_data; }

private:
    bool isInitialed = false;
    I2C_State m_state = I2C_State::IDLE;

    static Data sm_data;

    // 阻塞式读写，仅供 Init 初始化阶段使用
    void WriteReg(uint8_t reg, uint8_t val);
    uint8_t ReadReg(uint8_t reg);
};