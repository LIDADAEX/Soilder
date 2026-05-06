#include "BMI088.h"
#include <stdio.h>

bool BMI088::Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gPort, uint16_t gPin, GPIO_TypeDef* aPort, uint16_t aPin, TIM_HandleTypeDef* htim) {
    m_hspi = hspi; m_gPort = gPort; m_gPin = gPin; m_aPort = aPort; m_aPin = aPin;m_htim = htim;
    HAL_GPIO_WritePin(gPort, gPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(aPort, aPin, GPIO_PIN_SET);

    // Gyro 配置
    WriteReg(false, 0x14, 0xB6); 
    HAL_Delay(30); // 陀螺仪复位建议等待 30ms
    WriteReg(false, 0x10, 0x03); // 1000Hz ODR
    
    // Accel 强行拉回 SPI 模式
    uint8_t dummy_rx;
    uint8_t dummy_tx = 0x80; 
    HAL_GPIO_WritePin(m_aPort, m_aPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(m_hspi, &dummy_tx, &dummy_rx, 1, 10);
    HAL_GPIO_WritePin(m_aPort, m_aPin, GPIO_PIN_SET);
    HAL_Delay(10); 
    
    WriteReg(true, 0x7C, 0x00); // BMI088_ACC_PWR_ACTIVE_MODE
    HAL_Delay(10);
    WriteReg(true, 0x7D, 0x04); // 进入正常模式 
	
    isInitialed = true;
    return true;
}

void BMI088::RequestGyroRead() {
    Struct_SPI_Manage_Object* obj = Get_SPI_Obj(m_hspi);
    if (!obj) return;
	while(m_hspi->State != HAL_SPI_STATE_READY);
    
    obj->Tx_Buffer[0] = 0x82; // Gyro 地址
    for (int i = 1; i < 7; i++) obj->Tx_Buffer[i] = 0xFF;
    SPI_Send_Receive_Data(m_hspi, m_gPort, m_gPin, 1, 7);
}

void BMI088::RequestAccelRead() {
    Struct_SPI_Manage_Object* obj = Get_SPI_Obj(m_hspi);
    if (!obj) return;
	while(m_hspi->State != HAL_SPI_STATE_READY);

    obj->Tx_Buffer[0] = 0x92; // Accel 地址
    for (int i = 1; i < 8; i++) obj->Tx_Buffer[i] = 0xFF;
    SPI_Send_Receive_Data(m_hspi, m_aPort, m_aPin, 1, 8);
}

void BMI088::RequestTempRead() {
    Struct_SPI_Manage_Object* obj = Get_SPI_Obj(m_hspi);
    if (!obj || m_hspi->State != HAL_SPI_STATE_READY) return;

    obj->Tx_Buffer[0] = 0xA2;
    for (int i = 1; i < 3; i++) obj->Tx_Buffer[i] = 0xFF;	
    // 修正: 读温度需要 3 字节 (1 Dummy + 2 数据)
    SPI_Send_Receive_Data(m_hspi, m_aPort, m_aPin, 1, 4);
}

void BMI088::SPI_TxRxCpltCallback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Length) {
    if (!isInitialed) return;

    if (Tx_Buffer[0] == 0x82) { 
        const float scale = 1.0f / 16.384f;
        m_data.gx = (int16_t)((Rx_Buffer[2] << 8) | Rx_Buffer[1]) * scale;
        m_data.gy = (int16_t)((Rx_Buffer[4] << 8) | Rx_Buffer[3]) * scale;
        m_data.gz = (int16_t)((Rx_Buffer[6] << 8) | Rx_Buffer[5]) * scale;
		//printf("{g}%.5f,%.5f,%.5f\n", m_data.gx, m_data.gy, m_data.gz);
    } 
    else if (Tx_Buffer[0] == 0x92) { 
		int16_t tempx = (int16_t)((Rx_Buffer[3] << 8) | Rx_Buffer[2]);
		int16_t tempy = (int16_t)((Rx_Buffer[5] << 8) | Rx_Buffer[4]);
		int16_t tempz = (int16_t)((Rx_Buffer[7] << 8) | Rx_Buffer[6]);
		
		m_data.ax = tempx * (6.0f / 32768.0f);
		m_data.ay = tempy * (6.0f / 32768.0f);
		m_data.az = tempz * (6.0f / 32768.0f);
		
		//printf("{a}%.5f,%.5f,%.5f\n", m_data.ax, m_data.ay, m_data.az);
    }
    else if (Tx_Buffer[0] == 0xA2) {
		
		int16_t temp = Rx_Buffer[2] * 8 + Rx_Buffer[3] / 32;
		if(temp >1023)
			temp = temp - 2048;
		
		const float scale = 0.125;
        m_data.temperature = temp * scale + 23;
		float error = targetTemp - m_data.temperature;
		
		if((error < 500) && (error > -500))
			Temp_inter += error * Temp_I;
		
		int32_t output = error * Temp_P * 1000 + Temp_inter * 1000;
		if(output < 0)
			output = 0;
		else if(output > 1000)
			output = 1000;
		
		__HAL_TIM_SET_COMPARE(m_htim, TIM_CHANNEL_1, output);
		bool dasd = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6);
		
		if(error < 1){
			isStable = true;
		}
    }
}

void BMI088::WriteReg(bool isAccel, uint8_t reg, uint8_t val) {
    uint8_t p[2] = {reg, val};
    GPIO_TypeDef* port = isAccel ? m_aPort : m_gPort;
    uint16_t pin = isAccel ? m_aPin : m_gPin;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(m_hspi, p, 2, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}