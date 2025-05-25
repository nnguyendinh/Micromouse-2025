#include "gyro.h"
#include "main.h"
#include "delay.h"

int gyro_initialized = 0;
float Gx_offset = 0;
float Gy_offset = 0;
float Gz_offset = 0;

uint8_t gyroInit(void) {
    if (gyro_initialized) return 1;

    I2C_HandleTypeDef *hi2c1_ptr = Get_I2C1_Ptr();

    HAL_StatusTypeDef ret;
    uint8_t temp;

    // establishing connection with the device
    ret = HAL_I2C_IsDeviceReady(hi2c1_ptr, (DEVICE_ADDRESS << 1) + 0, 1, 1000);
    if (ret != HAL_OK) return ret;

    // configuring power management
    temp = 0b00001000;
    ret = HAL_I2C_Mem_Write(hi2c1_ptr, (DEVICE_ADDRESS << 1) + 0, REG_USER_CTRL, 1, &temp, 1, 100);
    if (ret != HAL_OK) return ret;

    // configuring gyroscope
    temp = FS_GYRO_500;
    ret = HAL_I2C_Mem_Write(hi2c1_ptr, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_GYRO, 1, &temp, 1, 100);
    if (ret != HAL_OK) return ret;

    // configuring accelerometer
    temp = FS_ACC_2G;
    ret = HAL_I2C_Mem_Write(hi2c1_ptr, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_ACC, 1, &temp, 1, 100);
    if (ret != HAL_OK) return ret;

    gyro_initialized = 1;

    // calculating offsets
//    float Gx_readings[10] = { 0.0 };
//    float Gy_readings[10] = { 0.0 };
    float Gz_readings[100] = { 0.0 };
//    float Gx_sum = 0.0;
//    float Gy_sum = 0.0;
    float Gz_sum = 0.0;

    while(1) {
    	Gz_offset = 0;
    	Gz_sum = 0;
    	for (int i = 0; i < 100; ++i) {
			readGyro(&Gz_readings[i]);
	//        Gx_sum += Gx_readings[i];
	//        Gy_sum += Gy_readings[i];
			Gz_sum += Gz_readings[i];
			delayMicroseconds(10000);
		}

	//    Gx_offset = Gx_sum / 10.0;
	//    Gy_offset = Gy_sum / 10.0;
		Gz_offset = Gz_sum / 100.0;

		Gz_sum = 0;
		for (int i = 0; i < 100; ++i) {
			readGyro(&Gz_readings[i]);
			Gz_sum += Gz_readings[i];
			delayMicroseconds(10000);
		}
		if (Gz_sum / 100.0 < 0.1 && Gz_sum / 100.0 > -0.1) {
			break;
		}
    }



    return 1;
}

void readGyro(/*float* Gx , float* Gy, */float* Gz) {
    if (!gyro_initialized) {
        return;
    }

    I2C_HandleTypeDef *hi2c1_ptr = Get_I2C1_Ptr();

//    uint8_t data[6];
    uint8_t data[2];
    HAL_I2C_Mem_Read(hi2c1_ptr, (DEVICE_ADDRESS << 1) + 0, REG_GYRO_DATA, 1, data, 2, 100);

//    int16_t Gyro_X_RAW = ((int16_t)(data[0] << 8)) | data [1];
//    int16_t Gyro_Y_RAW = ((int16_t)(data[2] << 8)) | data [3];
//    int16_t Gyro_Z_RAW = ((int16_t)(data[4] << 8)) | data [5];
    int16_t Gyro_Z_RAW = ((int16_t)(data[0] << 8)) | data [1];

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

//    *Gx = Gyro_X_RAW/131.0 - Gx_offset;
//    *Gy = Gyro_Y_RAW/131.0 - Gy_offset;
//    *Gz = Gyro_Z_RAW/65.5f - Gz_offset;
    *Gz = Gyro_Z_RAW/65.8f - Gz_offset;

    return;
}
