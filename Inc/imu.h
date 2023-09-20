/*
 * imu.h
 *
 *  Created on: Sep 14, 2023
 *      Author: pmclin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_
#define DEVICE_ADDRESS 0x6B
#define CTRL2_G_ADDRESS 0x11
#define CTRL1_XL_ADDRESS 0x10
#define OUTX_L_XL_ADDRESS 0x28
#define OUTX_H_XL_ADDRESS 0x29
#define CTRL6_C_ADDRESS 0x15
#define CTRL7_G_ADDRESS 0x16

void IMU_Init();

#endif /* INC_IMU_H_ */
