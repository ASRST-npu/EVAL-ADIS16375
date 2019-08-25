#include "stm32f4xx.h"


void GPIO_Configuration(void);
void RCC_Configuration (void);
void SPI_Config(void);
void NVIC_Config(void);
void EXTI_Configuration(void);

void USART1_Config(void);
void USART6_Config(void);

void STM32_Configuration(void);

#define SPIx_CLK 					SPI2
#define SPIx								SPI2

#define SPIx_SCK_GPIO_PORT  GPIOB
#define SPIx_MISO_GPIO_PORT GPIOB
#define SPIx_MOSI_GPIO_PORT GPIOB
#define SPIx_CS_GPIO_PORT   GPIOB

#define SPIx_CS_SOURCE   GPIO_PinSource12
#define SPIx_SCK_SOURCE  GPIO_PinSource13
#define SPIx_MISO_SOURCE GPIO_PinSource14
#define SPIx_MOSI_SOURCE GPIO_PinSource15

#define SPIx_CS_AF			 GPIO_AF_SPI2
#define SPIx_SCK_AF			 GPIO_AF_SPI2
#define SPIx_MISO_AF		 GPIO_AF_SPI2
#define SPIx_MOSI_AF		 GPIO_AF_SPI2

#define SPIx_CS_PIN		   GPIO_Pin_12
#define SPIx_SCK_PIN		 GPIO_Pin_13
#define SPIx_MISO_PIN		 GPIO_Pin_14
#define SPIx_MOSI_PIN		 GPIO_Pin_15
