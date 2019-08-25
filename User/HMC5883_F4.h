
#include <stdint.h>

#define HMC5883_ADDR 0x3C

void STM32F407Tek_HMC5883_Init(void);

void STM32F407Tek_HMC5883_WriteOneByte(uint8_t ucRegAddr,uint8_t ucDataToWrite);

uint8_t STM32F407Tek_HMC5883_ReadOneByte(uint8_t ucRegAddr);

void STM32F407Tek_HMC5883_ReadThreeAxes(int16_t* nAxes_X,int16_t *nAxes_Y,int16_t *nAxes_Z);


