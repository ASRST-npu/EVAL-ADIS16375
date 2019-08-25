/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013 
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "configuration.h"
#include "stdio.h"
#include "datatypes.h"
#include "ADIS16375_F4.h"
#include "HMC5883_F4.h"
#include "AHRS.h"
#include "math.h"

int n;
int m;
extern float a_x,a_y,a_z;
extern float g_x,g_y,g_z;
extern float theta;
extern float psi;
extern float gama;
extern float Vx;
extern float Vy;
extern float Vz;
extern float X;
extern float Y;
extern float Z;
extern float flag;
/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLUE_LED_OFF GPIO_SetBits(GPIOC,GPIO_Pin_8)
#define BLUE_LED_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_8)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


uint16_t IRQ_n;
uint16_t st1=0,st2,st3;
uint8_t step=0;
uint16_t reg;

static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);
void UART_Send(uint16_t head, float x,float y,float z);
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data);  
void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	
/*karman*/


int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/
       startup_stm32f429_439xx.s/startup_stm32f401xx.s) before to branch to 
       application main. To reconfigure the default setting of SystemInit() 
       function, refer to system_stm32f4xx.c file
     */  
	STM32_Configuration();
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  /* Add your application code here */
  /* Insert 50 ms delay */
  Delay(100);
	
	printf(" Start!\r\n");
	
	//adis16375_turn2page(0x03);
		reg=adis16375_read(0x0000);
		printf("reg=%d\r\n",reg);
		reg=adis16375_read(0x0E00);
		printf("reg=%d\r\n",reg);
		reg=adis16375_read(0x1000);
		printf("reg=%d\r\n",reg);
		reg=adis16375_read(0x7E00);
		printf("reg=%d\r\n",reg);

	
  /* Infinite loop */
  while (1)
  {    
		//AHRSupdateIMU(g_x,g_y,g_z,a_x,a_y,a_z);
		BLUE_LED_ON;
			//UART_Send('a',a_x,a_y,a_z);
			//UART_Send('g',g_x,g_y,g_z);
			//UART_Send('f',theta,psi,gama);
		
		printf("g=%f,%f,%f\r\n",g_x,g_y,g_z);
		Delay(100);
		BLUE_LED_OFF;
  }
}

void UART_Send(uint16_t head,float x,float y,float z)
{
	uint8_t data_h,data_l,i;
	int16_t data;
	
	union union_item
	{
		unsigned char c[2];
		short s;
	}xx;
	
	float data_f[3];
	
	data_f[0] = x;
	data_f[1] = y;
	data_f[2] = z;

	if(head == 'a')		//Acceleration
	{
		UART_PutChar(USART1, 0xff);
		UART_PutChar(USART1, 0xfa);
		
		for(i=0;i<=2;i++)
		{
			xx.s = (int16_t)(data_f[i] * 1800);
						
			UART_PutChar(USART1,xx.c[1]);
			UART_PutChar(USART1,xx.c[0]);
		}
		UART_PutChar(USART1, 0x0d);
		UART_PutChar(USART1, 0x0a);
	}
	if(head =='g')	//Angular rates
	{
		UART_PutChar(USART1, 0xff);
		UART_PutChar(USART1, 0xfb);
		 
		for(i=0;i<=2;i++)
		{
			data = (int16_t)(data_f[i] * 72);

			data_l = data&0x00ff;
			data   = data>>8;
			data_h = data&0x00ff;
			
			UART_PutChar(USART1,data_h);
			UART_PutChar(USART1,data_l);
		}
		UART_PutChar(USART1, 0x0d);
		UART_PutChar(USART1, 0x0a);
	}
	if(head =='f')		//Altitude
	{
		UART_PutChar(USART1, 0xff);
		UART_PutChar(USART1, 0xfc);
		
		for(i=0;i<=2;i++)
		{
			data = (int16_t)(data_f[i] * 100);
			data_l = data&0x00ff;
			data   = data>>8;
			data_h = data&0x00ff;
			
			UART_PutChar(USART1,data_h);
			UART_PutChar(USART1,data_l);
		}
		UART_PutChar(USART1, 0x0d);
		UART_PutChar(USART1, 0x0a);
	}

		
}
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  
{  
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  
	USART_SendData(USARTx, Data);  
  //  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
}  
void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str)    
{    
    while (0 != *str)    
    {    
        UART_PutChar(USARTx, *str);    
        str++;    
    }    
} 


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);
  /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}

    return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
