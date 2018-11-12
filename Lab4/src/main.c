/**

******************************************************************************
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
typedef enum state{
	SHOWTEMP,
	SETPOINT,
	FANON
} state;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

ADC_HandleTypeDef    Adc_Handle;
ADC_ChannelConfTypeDef Adc_Channel;
ADC_AnalogWDGConfTypeDef Adc_Watchdog;

TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure, Tim4_OCInitStructure;
uint16_t TIM3_Prescaler;   
uint16_t TIM3_CCR;   //make it interrupt every 500 ms, halfsecond.


__IO uint32_t ADC1ConvertedValue=0;   //if declare it as 16t, it will not work.
char temperatureString[6] = {0};
char setPointString[6] = {0};


volatile double  setPoint=23.5;
uint16_t tempAboveSetPoint = 0;
uint16_t belowGood = 0;							// variable to make sure you are below setPoint for adequate time

double measuredTemp; 
int a; 



char lcd_buffer[6];    // LCD display buffer

uint16_t sel_pressed, up_pressed, down_pressed;
state FanState = SHOWTEMP;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void ADC_Config(void);
double ADCtoDegC(uint32_t val);
void displayTempString(void);
void displaySetPoint(void);
void TIM3_Config(void);
void TIM3_OC_Config(void);

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

	sel_pressed=0;
	up_pressed=0;
	down_pressed=0;
	
	HAL_Init();

	SystemClock_Config();   

	HAL_InitTick(0x0000); // set systick's priority to the highest.

	TIM3_CCR = 20000;
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);

	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);  

	ADC_Config();
 
	TIM3_Config();

	TIM3_OC_Config();

  while (1)
  {
		if (sel_pressed==1) {
			//BSP_LED_Toggle(LED5);
			if (FanState == SETPOINT) {
				FanState = SHOWTEMP;
			}
			else {
				FanState = SETPOINT;
			}
			sel_pressed=0;
		}
		
		if (up_pressed==1) {
			//BSP_LED_Toggle(LED5);
			if (FanState == SETPOINT) {
				setPoint += 0.5;
			}
			up_pressed=0;
		}
		
		if (down_pressed==1) {
			//BSP_LED_Toggle(LED5);
			if (FanState == SETPOINT) {
				setPoint -= 0.5;
			}
			down_pressed=0;
		}
		
		switch (FanState) {
			case SHOWTEMP:
				displayTempString();
				if (tempAboveSetPoint == 1) {
					FanState = FANON;
					tempAboveSetPoint = 0;
				}
				break;
			case SETPOINT:
				displaySetPoint();
				break;
			case FANON:
				if (ADC1ConvertedValue >= (setPoint * 1/0.02442)) {	// To maker sure that one dip below setPoint doesn't retain belowGood value
					belowGood=0;
				}
				if (ADC1ConvertedValue < (setPoint * 1/0.02442)) {
					belowGood++;
				}
				if (belowGood >= 10) {
					belowGood = 0;
					FanState = SHOWTEMP;
				}
				break;
		}
	
	} //end of while 1

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

 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
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

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}

double ADCtoDegC(uint32_t val)
{
	return (0.02442*val);
}

void displayTempString(void)
{	
	measuredTemp = ADCtoDegC(ADC1ConvertedValue);
	sprintf(temperatureString, "%.1f", measuredTemp);
	BSP_LCD_GLASS_DisplayString((uint8_t*)temperatureString);	
}

void displaySetPoint(void)
{
	sprintf(lcd_buffer,"%.1f",setPoint);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
}

void ADC_Config(void)
{
	Adc_Handle.Instance = ADC1;
	if (HAL_ADC_DeInit(&Adc_Handle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }
	
  Adc_Handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  Adc_Handle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  Adc_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  Adc_Handle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  Adc_Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  Adc_Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  Adc_Handle.Init.ContinuousConvMode    = DISABLE;                        /* Continuous mode disabled (automatic conversion restart after each conversion) */
  Adc_Handle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
	Adc_Handle.Init.ExternalTrigConv			= ADC_SOFTWARE_START;           /* Software start to trig the 1st conversion manually, without external event */
  Adc_Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	Adc_Handle.Init.DMAContinuousRequests = ENABLE;                        /* DMA circular mode selected */
  Adc_Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  Adc_Handle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */
	
	  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&Adc_Handle) != HAL_OK)
  {
		BSP_LCD_GLASS_DisplayString((uint8_t*)"InitX");
    Error_Handler();
  }
  
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&Adc_Handle, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
		BSP_LCD_GLASS_DisplayString((uint8_t*)"CaliX");
    Error_Handler();
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  Adc_Channel.Channel      = ADC_CHANNEL_7;                /* Sampled channel number */
  Adc_Channel.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  Adc_Channel.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  Adc_Channel.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  Adc_Channel.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  Adc_Channel.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&Adc_Handle, &Adc_Channel) != HAL_OK)
  {
		BSP_LCD_GLASS_DisplayString((uint8_t*)"ConfX");
    Error_Handler();
  }
	/*### - 4 - Set up WatchDog ############################################### */
  /* Analog watchdog 1 configuration */
  Adc_Watchdog.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  Adc_Watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
  Adc_Watchdog.Channel = ADC_CHANNEL_7;
  Adc_Watchdog.ITMode = ENABLE;
  Adc_Watchdog.HighThreshold = (setPoint * 1/0.02442);
  if (HAL_ADC_AnalogWDGConfig(&Adc_Handle, &Adc_Watchdog) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

	
  /* ### - 5 - Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start_DMA(&Adc_Handle, (uint32_t*)&ADC1ConvertedValue, 1) != HAL_OK)
  {
		BSP_LCD_GLASS_DisplayString((uint8_t*)"StrtX");
    Error_Handler();
  }
}

void TIM3_Config(void)
{
	TIM3_Prescaler = (uint16_t)(SystemCoreClock/10000) - 1;		//10KHz
	
	Tim3_Handle.Instance = TIM3;
	Tim3_Handle.Init.Period = 40000 - 1;			//40000/10000 = 4s. This ensures WaitTime is 4s max
	Tim3_Handle.Init.Prescaler = TIM3_Prescaler;	
	Tim3_Handle.Init.ClockDivision = 0;
	Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
}

void TIM3_OC_Config(void)
{
	Tim3_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
	Tim3_OCInitStructure.Pulse = TIM3_CCR;		//20000/10000 = 0.5s
	Tim3_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	
	HAL_TIM_OC_Init(&Tim3_Handle);
	
	HAL_TIM_OC_ConfigChannel(&Tim3_Handle,&Tim3_OCInitStructure,TIM_CHANNEL_1);
	
	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1);
					
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
				sel_pressed=1;
				break;	
			case GPIO_PIN_1:     //left button						
					
				break;
			//case GPIO_PIN_2:    //right button				ADC using pin  PA2
			//			
			//				break;
			case GPIO_PIN_3:    //up button							
				up_pressed=1;
				break;
			case GPIO_PIN_5:    //down button						
				down_pressed=1;
				break;
			default://
						//default
						break;
	  } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer3 
	BSP_LED_Toggle(LED5);
	HAL_ADC_Start_DMA(&Adc_Handle,(uint32_t*)&ADC1ConvertedValue,1);
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
	
	__HAL_TIM_SET_COUNTER(htim, 0x0000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{

}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Set variable to report analog watchdog out of window status to main      */
  /* program.                                                                 */
  tempAboveSetPoint = 1;
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
