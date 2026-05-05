#pragma once

#include "1_Middleware/1_Driver/SPI/drv_spi.h"
#include "attitude_estimator.h"

class BMI088 {
public:
    struct Data {
        float gx, gy, gz;
        float ax, ay, az;
        float roll, pitch, yaw;
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
	void TIM_100ms_Callback();

    // 适配你驱动格式的回调函数
    void SPI_TxRxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length);
    Data GetData() { return sm_data; }

private:
	bool isInitialed = false;

	float gyro_bias_x = 0;
	float gyro_bias_y = 0;
	float gyro_bias_z = 0;

    static Data sm_data;
    static Class_Attitude_Estimator sm_attitude;

	static uint8_t sm_gyro_tx_buf; 
    static uint8_t sm_accel_tx_buf;
    static uint8_t sm_temp_tx_buf;

    void WriteReg(bool isAccel, uint8_t reg, uint8_t val);
};
