#pragma once

#include "1_Middleware/1_Driver/SPI/drv_spi.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"

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
	TIM_HandleTypeDef *m_htim;
	
	const float targetTemp = 45;
	const float Temp_P = 0.1;
	const float Temp_I = 0.01;
	float Temp_inter = 0;

    bool Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gPort, uint16_t gPin, GPIO_TypeDef* aPort, uint16_t aPin, TIM_HandleTypeDef *htim);
    void RequestGyroRead();
    void RequestAccelRead();
    void RequestTempRead(); // 之前你的 TIM_100ms_Callback

    void SPI_TxRxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length);
    Data GetData() { return m_data; }
    bool IsInitialed() const { return isInitialed; }
	bool IsStable() const {return isStable;}

private:
    bool isInitialed = false;
	bool isStable = false;
    Data m_data = {0};
	
    void WriteReg(bool isAccel, uint8_t reg, uint8_t val);
};