/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp_template.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    21-April-2017
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied 
  *          to the user folder.
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
#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL MSP module driver
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef Tim3_Handle, Tim4_Handle;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitialize the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

/**
  * @brief  Initialize the PPP MSP.
  * @param  None
  * @retval None
  */
void HAL_PPP_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitialize the PPP MSP.
  * @param  None  
  * @retval None
  */
void HAL_PPP_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit (TIM_HandleTypeDef *htim)
{
	if(htim == &Tim3_Handle)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIMx Peripheral clock enable */
		//
		__HAL_RCC_TIM3_CLK_ENABLE();
	
	
		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		//
		HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
	
	
		/* Enable the TIMx global Interrupt */
		//
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if(htim == &Tim4_Handle)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIMx Peripheral clock enable */
		//
		__HAL_RCC_TIM4_CLK_ENABLE();
	
	
		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		//
		HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
	
	
		/* Enable the TIMx global Interrupt */
		//
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}
}

//configure the 
void HAL_TIM_OC_MspInit (TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
//
	__HAL_RCC_TIM5_CLK_ENABLE();
	
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
//
	HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
  
  /* Enable the TIMx global Interrupt */
//
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

 void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
 {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /*Select PLLQ output as RNG clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	 
	 __HAL_RCC_RNG_CLK_ENABLE();
 }

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
