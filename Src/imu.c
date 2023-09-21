/*
 * imu.c
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */


#include <imu.h>
#include <main.h>

extern I2C_HandleTypeDef hi2c1;
float acc_x_offset;
float acc_y_offset;
float acc_z_offset;
float gyro_x_offset;
float gyro_y_offset;
float gyro_z_offset;
void IMU_Init()
{
	//Checks to see IMU and Microcontroller is connected
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, hi2c1.Init.OwnAddress1,  100, 1000);
	SENSOR_Config();
	FIFO_Config();
	ReadAccelerometerAtRest();
	READ_DATA();
}

void SENSOR_Config()
{
	// turn on accelerometer sensor high performance mode
	uint8_t Acc_Data = 10100100;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL1_XL_ADDRESS, 1, &Acc_Data, 1, 1000);
	uint8_t Acc_Control = 10000000;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL6_C_ADDRESS, 1, &Acc_Control, 1, 100);

//	turn on Gyroscope sensor high performance
	uint8_t Gyro_Data = 10101100;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL2_G_ADDRESS, 1, &Gyro_Data, 1, 1000);
	uint8_t Gyro_Control = 0b0000000;
	HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, CTRL7_G_ADDRESS, 1, &Gyro_Control, 1, 1000);
}


void FIFO_Config()
{

	uint8_t FIFO_CTRL3 = 00010010;
	HAL_StatusTypeDef FIFO_CTRL3_Result = HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, FIFO_CTRL3_ADDRESS, 1, &FIFO_CTRL3, 1, 1000);

	uint8_t FIFO_CTRL4 = 00010010;
	HAL_StatusTypeDef FIFO_CTRL4_Result = HAL_I2C_Mem_Write(&hi2c1, hi2c1.Init.OwnAddress1, FIFO_CTRL4_ADDRESS, 1, &FIFO_CTRL4, 1, 1000);
}

void READ_DATA()
{

	#define MAX_DATA_POINTS 100 // Adjust this based on your requirements

	float acc_x_data[MAX_DATA_POINTS];
	float acc_y_data[MAX_DATA_POINTS];
	float acc_z_data[MAX_DATA_POINTS];

	float gyro_x_data[MAX_DATA_POINTS];
	float gyro_y_data[MAX_DATA_POINTS];
	float gyro_z_data[MAX_DATA_POINTS];

	int data_index = 0;
	while(data_index != MAX_DATA_POINTS) {
		//Linear acceleration sesitivity: FS +-16 is .488
		//Angular rate sensitivity: FS = 2000 is 70

		//Read Accelerometer X
		uint8_t Acc_X_L[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, &Acc_X_L[0], 1, 100);
		uint8_t Acc_X_H[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, &Acc_X_H[0], 1, 100);
		uint16_t acc_x_raw = Acc_X_L[0] | (Acc_X_H[0] << 8);
		float acc_x = ((acc_x_raw *.488) / 1000) - acc_x_offset;
		//Read Accelerometer Y
		uint8_t Acc_Y_L[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, &Acc_Y_L[0], 1, 100);
		uint8_t Acc_Y_H[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, &Acc_Y_H[0], 1, 100);
		uint16_t acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
		float acc_y = ((acc_y_raw *.488) / 1000) - acc_y_offset;
		//Read Accelerometer Z
		uint8_t Acc_Z_L[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, &Acc_Z_L[0], 1, 100);
		uint8_t Acc_Z_H[1];
		HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, &Acc_Z_H[0], 1, 100);
		uint16_t acc_z_raw = Acc_Z_L[0] | (Acc_Z_H[0] << 8);
		float acc_z = ((acc_z_raw *.488) / 1000) - acc_z_offset;


		//Read Gyroscope X
		uint8_t Gyro_X_L[1];
		HAL_StatusTypeDef  X = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, &Gyro_X_L[0], 1, 100);
		uint8_t Gyro_X_H[1];
		HAL_StatusTypeDef X2 = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, &Gyro_X_H[0], 1, 100);
		uint16_t gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
		float gyro_x = (gyro_x_raw * 70) - gyro_x_offset;
		//Read Gyroscope Y
		uint8_t Gyro_Y_L[1];
		HAL_StatusTypeDef Y = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, &Gyro_Y_L[0], 1, 100);
		uint8_t Gyro_Y_H[1];
		HAL_StatusTypeDef Y2 = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, &Gyro_Y_H[0], 1, 100);
		uint16_t gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
		float gyro_y = (gyro_y_raw * 70) - gyro_y_offset;
		//Read Gyroscope Z
		uint8_t Gyro_Z_L[1];
		HAL_StatusTypeDef Z = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, &Gyro_Z_L[0], 1, 100);
		uint8_t Gyro_Z_H[1];
		HAL_StatusTypeDef Z2 = HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, &Gyro_Z_H[0], 1, 100);
		uint16_t gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
		float gyro_z = (gyro_z_raw * 70) - gyro_z_offset;

	    // Store the data
	    acc_x_data[data_index] = acc_x;
	    acc_y_data[data_index] = acc_y;
	    acc_z_data[data_index] = acc_z;

	    gyro_x_data[data_index] = gyro_x;
	    gyro_y_data[data_index] = gyro_y;
	    gyro_z_data[data_index] = gyro_z;

	    // Increment data_index (wrap around if it exceeds MAX_DATA_POINTS)
	    data_index = (data_index + 1);
	}

}

void ReadAccelerometerAtRest() {
    //Offset X
	uint8_t Acc_X_L[1];
    uint8_t Acc_X_H[1];
    uint16_t acc_x_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_XL_ADDRESS, 1, Acc_X_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_XL_ADDRESS, 1, Acc_X_H, 1, 100);
    acc_x_raw = Acc_X_L[0] | (Acc_X_H[0] << 8);
    acc_x_offset = (acc_x_raw * 0.488) / 1000;
    //Offset Y
    uint8_t Acc_Y_L[1];
    uint8_t Acc_Y_H[1];
    uint16_t acc_y_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_XL_ADDRESS, 1, Acc_Y_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_XL_ADDRESS, 1, Acc_Y_H, 1, 100);
    acc_y_raw = Acc_Y_L[0] | (Acc_Y_H[0] << 8);
    acc_y_offset = (acc_y_raw * 0.488) / 1000;

    //Offset Z
    uint8_t Acc_Z_L[1];
    uint8_t Acc_Z_H[1];
    uint16_t acc_z_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_XL_ADDRESS, 1, Acc_Z_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_XL_ADDRESS, 1, Acc_Z_H, 1, 100);
    acc_z_raw = Acc_Z_L[0] | (Acc_Z_H[0] << 8);
    acc_z_offset = (acc_z_raw * 0.488) / 1000;

    //Gyroscope
    //Offset X
	uint8_t Gyro_X_L[1];
    uint8_t Gyro_X_H[1];
    uint16_t gyro_x_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_L_G_ADDRESS, 1, Acc_X_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTX_H_G_ADDRESS, 1, Acc_X_H, 1, 100);
    gyro_x_raw = Gyro_X_L[0] | (Gyro_X_H[0] << 8);
    gyro_x_offset = (gyro_x_raw * 0.488) / 1000;

    //Offset Y
	uint8_t Gyro_Y_L[1];
    uint8_t Gyro_Y_H[1];
    uint16_t gyro_y_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_L_G_ADDRESS, 1, Acc_Y_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTY_H_G_ADDRESS, 1, Acc_Y_H, 1, 100);
    gyro_y_raw = Gyro_Y_L[0] | (Gyro_Y_H[0] << 8);
    gyro_y_offset = (gyro_y_raw * 0.488) / 1000;

    //Offset Z
	uint8_t Gyro_Z_L[1];
    uint8_t Gyro_Z_H[1];
    uint16_t gyro_z_raw;
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_L_G_ADDRESS, 1, Acc_Z_L, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, hi2c1.Init.OwnAddress1, OUTZ_H_G_ADDRESS, 1, Acc_Z_H, 1, 100);
    gyro_z_raw = Gyro_Z_L[0] | (Gyro_Z_H[0] << 8);
    gyro_z_offset = (gyro_z_raw * 0.488) / 1000;
}
