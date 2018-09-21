/**
  @page EEPROM_Emulation application to show an eeprom emulation

  @verbatim
  ******************************************************************************
  * @file    EEPROM_Emulation/readme.txt 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   Description of the EEPROM_Emulation application.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. All rights reserved.
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
  @endverbatim

@par Application Description 

This application is a software solution that allows substituting a standalone EEPROM 
by emulating the EEPROM mechanism using the on-chip Flash devices. The emulation
is achieved by using at least two Flash sectors and the coherence mechanism is ensured 
by swapping between both sectors.

Please refer to AN3969 for further details regarding this application.

STM32L496ZG-Nucleo (144 pins)'s LEDs can be used to monitor the application status:
  - LED1 is toggle when the example runs successfully.
  - LED3 is toggle in case of error.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 


 - EEPROM_Emulation/Inc/stm32l4xx_hal_conf.h        HAL Configuration file
 - EEPROM_Emulation/Inc/eeprom.h                      Header for main.c module 
 - EEPROM_Emulation/Inc/stm32l4xx_it.h              Header for stm32l4xx_it.c
 - EEPROM_Emulation/Src/main.c                      Main program
 - EEPROM_Emulation/Src/eeprom.c                      EEPROM program
 - EEPROM_Emulation/Src/stm32l4xx_it.c              Interrupt handlers
 - EEPROM_Emulation/Src/system_stm32l4xx.c          STM32L4xx system clock configuration file 
    

@par Hardware and Software environment 

  - This example runs on STM32L496ZG devices.
    
  - This example has been tested with STM32L496ZG-Nucleo (144 pins) board and can be
    easily tailored to any other supported device and development board. 

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
