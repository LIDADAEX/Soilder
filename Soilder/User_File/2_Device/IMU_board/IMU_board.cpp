#include "IMU_board.h"
#include <algorithm>

void IMU_Board::Init() {
    imu.Init(&hspi1, GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_4);
    mag.Init(&hi2c3, GPIOG, GPIO_PIN_3, GPIOG, GPIO_PIN_6);
    estimator.Init();
}

void IMU_Board::Update(float dt) {
    if(!imu.IsInitialed()) return;

    auto data = imu.GetData();
    auto m = mag.GetData();

    // === 计算陀螺仪零偏 ===
    GyroBiasUpdate(data.gx, data.gy, data.gz, data.ax, data.ay, data.az);
	
    // 减去零偏 (注意这里的单位是 deg/s)
    float clean_gx = data.gx;
    float clean_gy = data.gy;
    float clean_gz = data.gz;

	if(bcnt < 1000) return;
	
	clean_gx -= bgx/bcnt;
	clean_gy -= bgy/bcnt;
	clean_gz -= bgz/bcnt;
	
    // === 送入姿态解算器 ===
    // 剥离出了驱动，在板级层面进行数据融合
    estimator.Update(data.ax, data.ay, data.az, 
                     clean_gx, clean_gy, clean_gz, 
                     m.mx, m.my, m.mz, 
                     dt);

    // === 获得结果 ===
    roll  = estimator.Roll;
    pitch = estimator.Pitch;
    yaw   = estimator.Yaw;
}

void IMU_Board::GyroBiasUpdate(float gx, float gy, float gz, float ax, float ay, float az){
    if(std::max({ax, ay}) <= 0.3  && std::max({gx, gy, gz}) <= 0.05){
        bgx += gx;
        bgy += gy;
        bgz += gz;
        bcnt ++;
    }
}