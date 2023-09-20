/*
 * imu.c
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */


#include <imu.h>
#include <main.h>

extern I2C_HandleTypeDef hi2c1;
void IMU_Init()
{
	//Checks to see IMU and Microcontroller is connected
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, hi2c1.Init.OwnAddress1,  100, 1000);

	// turn on accelerometer sensor high performance mode
	uint8_t Data = 10100100;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL1_XL_ADDRESS, 1, &Data, 1, 1000);
	uint8_t AccelControl = 10000000;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL6_C_ADDRESS, 1, &AccelControl, 1, 100);

//	turn on Gyroscope sensor high performance
	uint8_t Data2 = 10101100;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL2_G_ADDRESS, 1, &Data2, 1, 1000);
	uint8_t GyroControl = 0b0000000;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL7_G_ADDRESS, 1, &GyroControl, 1, 1000);


	while(1) {
		uint8_t Data7[1];
		uint8_t x_acc_l = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, &Data2, 1, 100);
		uint8_t Data3[1];
		uint8_t x_acc_h = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, &Data3, 1, 100);
		printf("");
	}


}
