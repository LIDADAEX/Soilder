#include "BMI088.h"

// 静态成员变量初始化
BMI088::Data BMI088::sm_data = {0};
Class_Attitude_Estimator BMI088::sm_attitude;
uint8_t BMI088::sm_gyro_tx_buf = 0x82;
uint8_t BMI088::sm_accel_tx_buf = 0x92;
uint8_t BMI088::sm_temp_tx_buf = 0xA2;

// ... 构造函数 ...

bool BMI088::Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gPort, uint16_t gPin, GPIO_TypeDef* aPort, uint16_t aPin) {

    m_hspi = hspi;
    m_gPort = gPort; 
    m_gPin = gPin;
    m_aPort = aPort; 
    m_aPin = aPin;
	
	HAL_GPIO_WritePin(gPort, gPin, GPIO_PIN_SET);

    sm_attitude.Init();  // 初始化解算器和滤波器系数

	
	
    // 硬件配置 (1kHz 采样率设定)-
    WriteReg(false, 0x14, 0xB6); 
	HAL_Delay(10);
	
    WriteReg(false, 0x0F, 0x00); // 2000dps
    WriteReg(false, 0x10, 0x02); // 1000Hz ODR
    
    // --- 加速度计配置 ---
    // 增加：任意进行一次读取操作，产生CS下降沿，将加速度计从I2C强行拉回SPI模式
    uint8_t dummy_rx;
    uint8_t dummy_tx = 0x00 | 0x80; // 读取0x00寄存器
    HAL_GPIO_WritePin(m_aPort, m_aPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(m_hspi, &dummy_tx, &dummy_rx, 1, 10);
    HAL_GPIO_WritePin(m_aPort, m_aPin, GPIO_PIN_SET);
    HAL_Delay(10); // 给它一点时间稳定
	
	WriteReg(true, 0x7C, 0x00); // BMI088_ACC_PWR_ACTIVE_MODE
	HAL_Delay(10);

    WriteReg(true, 0x7D, 0x04); // 现在才能真正写入正常模式
    HAL_Delay(1000);

	for (int i = 0; i < 1000; i++) {
		RequestGyroRead();
		gyro_bias_x += sm_data.gx;
		gyro_bias_y += sm_data.gy;
		gyro_bias_z += sm_data.gz;
		HAL_Delay(1);
	}

	gyro_bias_x /= 1000;
	gyro_bias_y /= 1000;
	gyro_bias_z /= 1000;
	
	isInitialed = true;

    return true;
}

void BMI088::RequestGyroRead() {
    Struct_IIC_Manage_Object* obj = Get_SPI_Obj(m_hspi);
	if (!obj) return;
	while(m_hspi->State != HAL_SPI_STATE_READY);
	
    obj->Tx_Buffer[0] = 0x82; // 陀螺仪读地址

    for (int i = 1; i < 7; i++) {
        obj->Tx_Buffer[i] = 0xFF;  // dummy
    }

    // 调用你写的驱动：发送1字节地址，接收6字节数据
    SPI_Send_Receive_Data(m_hspi, m_gPort, m_gPin, 1, 7);
}

void BMI088::RequestAccelRead() {
    Struct_IIC_Manage_Object* obj = Get_SPI_Obj(m_hspi);
    if (!obj) return;
	while(m_hspi->State != HAL_SPI_STATE_READY);

    obj->Tx_Buffer[0] = 0x92;

    for (int i = 1; i < 8; i++) {
        obj->Tx_Buffer[i] = 0xFF;  // dummy
    }

    SPI_Send_Receive_Data(m_hspi, m_aPort, m_aPin, 1, 8);
}

void BMI088::TIM_100ms_Callback() {
    // 调用异步 SPI 传输：读取 3 字节温度数据 (包含 1 字节 Dummy)
    Struct_IIC_Manage_Object* obj = Get_SPI_Obj(m_hspi);
    if (!obj) return;

    obj->Tx_Buffer[0] = 0xA2; // 加速度计读地址
    SPI_Send_Receive_Data(m_hspi, m_aPort, m_aPin, 1, 3);
}

void BMI088::SPI_TxRxCpltCallback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Length) {
    if (Tx_Buffer[0] == 0x82) { // 处理陀螺仪[cite: 8]
        const float scale = 1.0f / 16.384f;
        // Rx[0]: Dummy(Address stage), Rx[1..2]: X, Rx[3..4]: Y, Rx[5..6]: Z
        sm_data.gx = (int16_t)((Rx_Buffer[2] << 8) | Rx_Buffer[1]) * scale ;
        sm_data.gy = (int16_t)((Rx_Buffer[4] << 8) | Rx_Buffer[3]) * scale ;
        sm_data.gz = (int16_t)((Rx_Buffer[6] << 8) | Rx_Buffer[5]) * scale ;

		if(isInitialed){
//			float acc_norm = sqrt(sm_data.ax*sm_data.ax + sm_data.ay*sm_data.ay + sm_data.az*sm_data.az);
//			
//			if (fabs(acc_norm - 1.0f) < 0.05f &&   // 接近1g
//				fabs(sm_data.gx) < 0.05f &&
//				fabs(sm_data.gy) < 0.05f &&
//				fabs(sm_data.gz) < 0.05f) 
//			{
//				gyro_bias_x = 0.995f * gyro_bias_x + 0.005f * sm_data.gx;
//				gyro_bias_y = 0.995f * gyro_bias_y + 0.005f * sm_data.gy;
//				gyro_bias_z = 0.995f * gyro_bias_z + 0.005f * sm_data.gz;
//			}
			
			sm_data.gx -= gyro_bias_x;
			sm_data.gy -= gyro_bias_y;
			sm_data.gz -= gyro_bias_z;
			
			if (fabs(sm_data.gx) < 0.03f) sm_data.gx = 0;
			if (fabs(sm_data.gy) < 0.03f) sm_data.gy = 0;
			if (fabs(sm_data.gz) < 0.03f) sm_data.gz = 0;
			
			// 触发算法更新[cite: 6]
			sm_attitude.Update(sm_data.ax, sm_data.ay, sm_data.az, 
							sm_data.gx, sm_data.gy, sm_data.gz, 0.001f);
			
			sm_data.roll  = sm_attitude.Roll;
			sm_data.pitch = sm_attitude.Pitch;
			sm_data.yaw   = sm_attitude.Yaw;
		}
    } 
    else if (Tx_Buffer[0] == 0x92) { // 处理加速度计[cite: 8]
        const float scale = 12.0f / 32768.0f;
        // Rx[0]: Dummy(Address stage), Rx[1]: Dummy(BMI088 Accel requirement)[cite: 8]
        // 真正数据从 Rx[2] 开始
		sm_data.ax = (int16_t)((Rx_Buffer[3] << 8) | Rx_Buffer[2]) * scale;
		sm_data.ay = (int16_t)((Rx_Buffer[5] << 8) | Rx_Buffer[4]) * scale;
		sm_data.az = (int16_t)((Rx_Buffer[7] << 8) | Rx_Buffer[6]) * scale;
    }
	else if (Tx_Buffer[0] == 0xA2) {
        sm_data.temperature = (int16_t)((Rx_Buffer[2] << 8) | Rx_Buffer[1]);
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