/*
 * gyro.h
 *
 *  Created on: May 16, 2024
 *      Author: parth
 */
/*
 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
*/

#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include "main.h"

#define DEVICE_ADDRESS         0x68

#define FS_GYRO_250            0
#define FS_GYRO_500            8
#define FS_GYRO_1000        16
#define FS_GYRO_2000        24

#define FS_ACC_2G            0
#define FS_ACC_4G            8
#define FS_ACC_8G            16
#define FS_ACC_16G            24

#define REG_CONFIG_GYRO     27
#define REG_CONFIG_ACC         28
#define REG_USER_CTRL         107
#define REG_GYRO_DATA         71
#define REG_ACC_DATA         59

uint8_t gyroInit(void);    // 1 if connection is established, 0 if no connection
void readGyro(/*float* Gx , float* Gy, */float* Gz);
// void readAcc(int16_t* Ax, int16_t* Ay, int16_t* Az);
uint8_t isGyroInitialized(void);
void recalculateOffset(void);

#endif /* INC_GYRO_H_ */