/**
  ******************************************************************************
  * @file    EEPROM_Emulation/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo_144.h"
#include "eeprom.h"

/** @addtogroup STM32L4xx_HAL_Applications
  * @{
  */

/** @addtogroup EEPROM_Emulation
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue,VarDataTmp;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  EE_Status ee_status;

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

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

  /* Configure LED1 & LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED3);
  
  /* EEPROM Init */
  ee_status = EE_Init();
  if( ee_status != EE_OK)  Error_Handler();

  /* --- Store successively many values of the three variables in the EEPROM ---*/
  /* Store 0x1000 values of Variable1 in EEPROM */
  for (VarValue = 1; VarValue <= 0x1000; VarValue++)
  {
    ee_status = EE_WriteVariable(VirtAddVarTab[0],  VarValue);
    ee_status|= EE_ReadVariable(VirtAddVarTab[0],  &VarDataTab[0]);
    if (VarValue != VarDataTab[0]) Error_Handler();
    
    ee_status|= EE_WriteVariable(VirtAddVarTab[1], ~VarValue);
    ee_status|= EE_ReadVariable(VirtAddVarTab[1],  &VarDataTab[1]);
    if(((uint16_t)~VarValue) != VarDataTab[1]) Error_Handler();

    ee_status|= EE_WriteVariable(VirtAddVarTab[2],  VarValue << 1);
    ee_status|= EE_ReadVariable(VirtAddVarTab[2],  &VarDataTab[2]);
    if ((VarValue << 1) != VarDataTab[2]) Error_Handler();
    
    if( ee_status != EE_OK)  Error_Handler();
  }

  /* Store 0x2000 values of Variable2 in EEPROM */
  for (VarValue = 1; VarValue <= 0x2000; VarValue++)
  {
    ee_status = EE_WriteVariable(VirtAddVarTab[1], VarValue);
    ee_status|= EE_ReadVariable(VirtAddVarTab[1], &VarDataTab[1]);
    
    if((VarValue != VarDataTab[1]) || (ee_status != EE_OK))
      Error_Handler();
  }

  /* read the last stored variables data*/
  ee_status = EE_ReadVariable(VirtAddVarTab[0], &VarDataTmp);
  if (VarDataTmp != VarDataTab[0]) Error_Handler();
  ee_status|= EE_ReadVariable(VirtAddVarTab[1], &VarDataTmp);
  if (VarDataTmp != VarDataTab[1]) Error_Handler();
  ee_status|= EE_ReadVariable(VirtAddVarTab[2], &VarDataTmp);
  if (VarDataTmp != VarDataTab[2]) Error_Handler();

  if( ee_status != EE_OK)  Error_Handler();
  
  /* Store 0x3000 values of Variable3 in EEPROM */
  for (VarValue = 1; VarValue <= 0x3000; VarValue++)
  {
    ee_status = EE_WriteVariable(VirtAddVarTab[2], VarValue);
    ee_status|= EE_ReadVariable(VirtAddVarTab[2], &VarDataTab[2]);
    
    if((VarValue != VarDataTab[2]) || (ee_status != EE_OK))
      Error_Handler();
  }

  /* read the last stored variables data*/
  ee_status = EE_ReadVariable(VirtAddVarTab[0], &VarDataTmp);
  if (VarDataTmp != VarDataTab[0]) Error_Handler();
  ee_status|= EE_ReadVariable(VirtAddVarTab[1], &VarDataTmp);
  if (VarDataTmp != VarDataTab[1]) Error_Handler();
  ee_status|= EE_ReadVariable(VirtAddVarTab[2], &VarDataTmp);
  if (VarDataTmp != VarDataTab[2]) Error_Handler();

  if( ee_status != EE_OK) Error_Handler();


  while (1)
  {
    /* Toggle LED1 fast */
    BSP_LED_Toggle(LED1);
    HAL_Delay(40);
  };
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
    /* Toggle LED3 fast */
    BSP_LED_Toggle(LED3);
    HAL_Delay(40);
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
