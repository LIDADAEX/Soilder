#pragma once

#include "1_Middleware/1_Driver/SPI/drv_spi.h"

class BMI088 {
public:
    struct Data {
        float gx, gy, gz; // deg/s
        float ax, ay, az; // g 或 m/s2
        float temperature;
    };

    SPI_HandleTypeDef* m_hspi;
    GPIO_TypeDef *m_gPort;
    uint16_t m_gPin;
    GPIO_TypeDef *m_aPort;
    uint16_t m_aPin;

    bool Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gPort, uint16_t gPin, GPIO_TypeDef* aPort, uint16_t aPin);
    void RequestGyroRead();
    void RequestAccelRead();
    void RequestTempRead(); // 之前你的 TIM_100ms_Callback

    void SPI_TxRxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length);
    Data GetData() { return m_data; }
    bool IsInitialed() const { return isInitialed; }

private:
    bool isInitialed = false;
    Data m_data = {0};

    void WriteReg(bool isAccel, uint8_t reg, uint8_t val);
};