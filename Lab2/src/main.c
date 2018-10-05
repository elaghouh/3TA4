/******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

char lcd_buffer[6];    // LCD display buffer
TIM_HandleTypeDef Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue, Tim4_PrescalerValue;
uint16_t Tim4_CCR;


__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, or HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type




RNG_HandleTypeDef Rng_Handle;






/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
	
	HAL_Init();

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();
 
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);

	BSP_LCD_GLASS_Init();
	
	TIM3_Config();
	
	TIM4_Config();
	
	Tim4_CCR = 10000;				// freqeuncy = 10 KHz. 10000/10000 = 1s
	
	TIM4_OC_Config();
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	//BSP_LCD_GLASS_ScrollSentence((uint8_t*) "  mt3ta4 lab2 starter", 1, 200);
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");
	







	
//******************* use emulated EEPROM ====================================
	//First, Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }
// then can write to or read from the emulated EEPROM
	

	
	
	
	
	
	
	
	
	
//*********************use RNG ================================  
Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
//then can use RNG
	
	

  /* Infinite loop */
  while (1)
  {
		

	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  // The following clock configuration sets the Clock configuration sets after System reset                
  // It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig 
  // and to be eventually adapted to new clock configuration                                               

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz


void TIM3_Config(void)
{
	Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/10000) - 1;     // To set counter clock to 10 KHz. Have to be careful because scaler must be <65,535
	
	Tim3_Handle.Instance = TIM3;
	
	Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
	Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim3_Handle.Init.Period = 10000 - 1;							// 1 second period bc 10KHz CLK. 10000/10000 = 1 s
	Tim3_Handle.Init.ClockDivision = 0;
	if (HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) {
		Error_Handler();
	}
	
	if (HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK) {
		Error_Handler();
	}
	
	
}

void TIM4_Config(void)
{
	Tim4_PrescalerValue = (uint16_t)(SystemCoreClock/10000) + 1;
	
	Tim4_Handle.Instance = TIM4;
	
	Tim4_Handle.Init.ClockDivision = 0;
	Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim4_Handle.Init.Period = 10000 - 1;
	Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
	
}

void TIM4_OC_Config(void)
{
	Tim4_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
	Tim4_OCInitStructure.Pulse = Tim4_CCR;
	Tim4_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	
	if(HAL_TIM_OC_Init(&Tim4_Handle) != HAL_OK) {
		Error_Handler();
	}
	
	if(HAL_TIM_OC_ConfigChannel(&Tim4_Handle,&Tim4_OCInitStructure,TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	
	if(HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						BSP_LCD_GLASS_ScrollSentence((uint8_t*)"You are a loser ", (uint16_t) 2, (uint16_t) 750);				
						break;	
			case GPIO_PIN_1:     //left button						
						
							break;
			case GPIO_PIN_2:    //right button						  to play again.
					
							break;
			case GPIO_PIN_3:    //up button							
					
							break;
			
			case GPIO_PIN_5:    //down button						
					
							break;
			default://
						//default
						break;
	  } 
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32lxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
	
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	BSP_LED_On(LED5);

		//clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h

}


static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
 
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
