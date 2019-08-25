/**
  ******************************************************************************
  * @file    adi375.c
  * @author  NERC
  * @version V1.1.0
  * @date    22-Mar-2017
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the ADIS16375 IMU Sensor:           
  *           + Initialization and Configuration
  *           + Motion read
  *           + Calibration
  * 
 ===============================================================================
                      ##### How to use this driver #####
 =============================================================================== 
1. 读部分：
	uint16_t adis16375_duplicate_read(uint16_t addr)  重复读取，CS不上拉
	uint16_t adis16375_read(uint16_t addr)						单次读取，CS上拉
	
2. 

*/
/* Includes ------------------------------------------------------------------*/
#include "ADIS16375_F4.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float a_x,a_y,a_z;
float g_x,g_y,g_z;
extern uint16_t st1,st2,st3;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
  *                      x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
  *                      x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */


//read
void Gyro_read(void)
{
	uint16_t reg_gx,reg_gy,reg_gz;
	uint16_t reg_ax,reg_ay,reg_az;
	
	reg_gx = adis16375_read(0x1200);
	reg_gy = adis16375_read(0x1600);
	reg_gz = adis16375_read(0x1A00);
	
	g_x = (short int)reg_gx;
	g_x = g_x * 0.013108f;
		
	g_y = (short int)reg_gy;
	g_y = g_y * 0.013108f;
	
	g_z = (short int)reg_gz;
	g_z = g_z * 0.013108f;
		
	reg_ax = adis16375_read(0x1E00);
	reg_ay = adis16375_read(0x2200);
	reg_az = adis16375_read(0x2600);
	
	a_x = (short int)reg_ax;
	a_x = a_x * 0.8192f * 0.001f;
	
	a_y = (short int)reg_ay;
	a_y = a_y * 0.8192f * 0.001f;
	
	a_z = (short int)reg_az;
	a_z = a_z * 0.8192f * 0.001f;
	
}

void Acc_read(void)
{
	uint16_t reg_ax,reg_ay,reg_az;
	
	reg_ax = adis16375_read(0x1E00);
	delay_nus(100);
	reg_ay = adis16375_read(0x2200);
	delay_nus(100);
	reg_az = adis16375_read(0x2600);
	delay_nus(100);
	
	a_x = (short int)reg_ax;
	a_x = a_x * 0.8192f * 0.001f;
	
	a_y = (short int)reg_ay;
	a_y = a_y * 0.8192f * 0.001f;
	
	a_z = (short int)reg_az;
	a_z = a_z * 0.8192f * 0.001f;
		
}

/*3.12?aadis16375_imu(), still have bug*/
void adis16375_imu_read(float acc[3], float gyro[3], float* Tempetature)
{
	uint16_t reg_ax,reg_ay,reg_az,reg_gx,reg_gy,reg_gz;
	uint16_t T;
	
	Select();
	delay_nus(20);
	adis16375_duplicate_read(0x0800);
	delay_nus(20);
	st1= adis16375_duplicate_read(0x0E00);
	delay_nus(20);
	T  = adis16375_duplicate_read(0x1200);
	delay_nus(20);
	reg_gx = adis16375_duplicate_read(0x1600);
	delay_nus(20);
	reg_gy = adis16375_duplicate_read(0x1A00);
	delay_nus(20);
	reg_gz = adis16375_duplicate_read(0x1E00);
	delay_nus(20);
	reg_ax = adis16375_duplicate_read(0x2200);
	delay_nus(20);
	reg_ay = adis16375_duplicate_read(0x2600);
	delay_nus(20);
	reg_az = adis16375_duplicate_read(0x0000);

	Deselect();
	
	gyro[0] = (short int)reg_gx;
	gyro[0] = gyro[0] * 0.013108f;
	
	gyro[1] = (short int)reg_gy;
	gyro[1] = gyro[1] * 0.013108f;
	
	gyro[2] = (short int)reg_gz;
	gyro[2] = gyro[2] * 0.013108f;
	
	acc[0] = (short int)reg_ax;
	acc[0] = acc[0] * 0.8192f * 0.001f;
	
	acc[1] = (short int)reg_ay;
	acc[1] = acc[1] * 0.8192f * 0.001f;
	
	acc[2] = (short int)reg_az;
	acc[2] = acc[2] * 0.8192f * 0.001f;
	
}
uint16_t adis16375_duplicate_read(uint16_t addr)
{
	uint16_t x;
	
	while (SPI_I2S_GetFlagStatus(imu_spi, SPI_I2S_FLAG_TXE) == RESET);
  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(imu_spi, addr);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(imu_spi, SPI_I2S_FLAG_RXNE) == RESET);
  /*!< Return the byte read from the SPI bus */
  x = SPI_I2S_ReceiveData(imu_spi);
	
	return x;
}

uint16_t adis16375_read(uint16_t addr)
{
  uint16_t y;
	
  Select();
	delay_nus(100);
	while (SPI_I2S_GetFlagStatus(imu_spi, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(imu_spi, addr);
	delay_nus(100);
  while (SPI_I2S_GetFlagStatus(imu_spi, SPI_I2S_FLAG_RXNE) == RESET);
  y = SPI_I2S_ReceiveData(imu_spi);
	delay_nus(100);
  Deselect();
	
  return y;
}

void adis16375_turn2page(uint8_t PAGEID)
{
	uint16_t adis375_pageid;
	
	adis375_pageid = PAGEID;
	adis375_pageid = adis375_pageid | 0x8000;
	
	Select();
	while (SPI_I2S_GetFlagStatus(imu_spi, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(imu_spi, adis375_pageid);
	Deselect();
}

void adis16375_write(uint8_t addr, uint8_t reg)
{
	uint16_t adis375_reg;
	
	adis375_reg = (uint16_t)addr;
	adis375_reg = adis375_reg<<8;
	adis375_reg = adis375_reg | reg;
	adis375_reg = adis375_reg | 0x8000;
	
	Select();
	
	while (SPI_I2S_GetFlagStatus(imu_spi, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(imu_spi, adis375_reg);

  Deselect();
}

void delay_nus(uint16_t x)
{
	while (x>0)
	{
		x=x-1;
	}
	
}


