#include "datatypes.h"
#include "configuration.h"


void Gyro_read (void);
void Acc_read(void);

void adis16375_imu_read(float acc[3], float gyro[3], float* Tempetature);
uint16_t adis16375_read(uint16_t addr);
uint16_t adis16375_duplicate_read(uint16_t addr);

void adis16375_turn2page(uint8_t PAGEID);
void adis16375_write(uint8_t addr,uint8_t reg);

void delay_nus(uint16_t x);

#define imu_spi SPI2

#define Select()   GPIO_ResetBits(GPIOB,GPIO_Pin_12);
#define Deselect() GPIO_SetBits(GPIOB,GPIO_Pin_12);

#define PAGE_ADD 0x00
#define PAGE_00     0x00
#define PAGE_02     0x02
#define PAGE_03     0x03

#define SYS_E_FLAG  0x08
#define DIAG_STS    0x0A
#define ALM_STS     0x0C
#define TEMP_OUT    0x0E

#define X_GYRO_LOW  0x10
#define X_GYRO_OUT  0x12
#define Y_GYRO_LOW  0x14
#define Y_GYRO_OUT  0x16
#define Z_GYRO_LOW  0x18
#define Z_GYRO_OUT  0x1A
#define X_ACCL_LOW  0x1C
#define X_ACCL_OUT  0x1E
#define Y_ACCL_LOW  0x20
#define Y_ACCL_OUT  0x22
#define Z_ACCL_LOW  0x24
#define Z_ACCL_OUT  0x26

#define Z_ACCL_FAULT 0x20
#define Y_ACCL_FAULT 0x10
#define X_ACCL_FAULT 0x08
#define Z_GYRO_FAULT 0x04
#define Y_GYRO_FAULT 0x02
#define X_GYRO_FAULT 0x01

#define GLOB_CMD    0x02

