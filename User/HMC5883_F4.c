
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "HMC5883_F4.h"

#define IMU_I2C I2C2

void HMC5883_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_I2C2);

	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed=200000;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1=HMC5883_ADDR;
	I2C_Init(IMU_I2C,&I2C_InitStructure);

	I2C_Cmd(IMU_I2C,ENABLE);
}

void HMC5883_WriteOneByte(uint8_t ucRegAddr,uint8_t ucDataToWrite)
{
	while(I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BUSY));

	I2C_GenerateSTART(IMU_I2C,ENABLE);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(IMU_I2C,HMC5883_ADDR,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(IMU_I2C,ucRegAddr);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(IMU_I2C,ucDataToWrite);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(IMU_I2C,ENABLE);

}

uint8_t HMC5883_ReadOneByte(uint8_t ucRegAddr)
{
	uint8_t ucDataRead=0;

	while(I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BUSY));

	I2C_GenerateSTART(IMU_I2C,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(IMU_I2C,HMC5883_ADDR,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(IMU_I2C,ucRegAddr);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(IMU_I2C,ENABLE);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(IMU_I2C,HMC5883_ADDR,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	I2C_AcknowledgeConfig(IMU_I2C,DISABLE);
	while(!I2C_CheckEvent(IMU_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED));
	ucDataRead=I2C_ReceiveData(IMU_I2C);

	I2C_GenerateSTOP(IMU_I2C,ENABLE);

	return ucDataRead;
}

void HMC5883_ReadThreeAxes(int16_t* nAxis_X,int16_t *nAxis_Y,int16_t *nAxis_Z)
{
		uint8_t ucHigh=0,ucLow=0;

		ucHigh=HMC5883_ReadOneByte(0x03);
		ucLow =HMC5883_ReadOneByte(0x04);
		(*nAxis_X)=ucHigh;
		(*nAxis_X)<<=8;
		(*nAxis_X)|=ucLow;

		ucHigh=HMC5883_ReadOneByte(0x05);
		ucLow =HMC5883_ReadOneByte(0x06);
		(*nAxis_Z)=ucHigh;
		(*nAxis_Z)<<=8;
		(*nAxis_Z)|=ucLow;

		ucHigh=HMC5883_ReadOneByte(0x07);
		ucLow =HMC5883_ReadOneByte(0x08);
		(*nAxis_Y)=ucHigh;
		(*nAxis_Y)<<=8;
		(*nAxis_Y)|=ucLow;
}
