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


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

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
typedef enum SystemState{TIMESHOW,DATESHOW,PRESSSHOW,TIMEDATESET,PRESSRECORD} SystemState;
typedef enum timedatesetpos{SECOND_POS=0,MINUTE_POS=1,HOUR_POS=2,DATE_POS=3,DAY_POS=4,MONTH_POS=5,YEAR_POS=6} timedatesetpos;
SystemState state = TIMESHOW;

I2C_HandleTypeDef  pI2c_Handle;
HAL_StatusTypeDef EE_status;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

//memory location to write to in the device
__IO uint16_t memLocation; //pick any location within range

  

char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};  //   
char datestring[12]={0};


uint8_t wd=0x00, dd=0x00, mo=0x0A, yy=0x18, ss=0x00, mm=0x00, hh=0x00; // for weekday, day, month, year, second, minute, hour

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed, push1pressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while (>800ms)
timedatesetpos settingpos = SECOND_POS;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void Pushbutton1_Init(void);
void RTC_Config(void);
void RTC_AlarmAConfig(void);
void RTC_DateShow(RTC_HandleTypeDef *hrtc);

HAL_StatusTypeDef EE_RecordTime(RTC_HandleTypeDef *hrtc);
void EE_DisplayTime(RTC_HandleTypeDef *hrtc);


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

	leftpressed=0;
	rightpressed=0;
	uppressed=0;
	downpressed=0;
	selpressed=0;
	sel_held=0;
	push1pressed=0;
	

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()

	
	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	Pushbutton1_Init();

	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	HAL_Delay(1000);


//configure real-time clock
	RTC_Config();
	
	RTC_AlarmAConfig();
	
	I2C_Init(&pI2c_Handle);

	memLocation=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,0x0000);
	HAL_Delay(1000);

	//I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, 0x0000, 0x0001);

/*********************Testing I2C EEPROM------------------

	//the following variables are for testing I2C_EEPROM
	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	uint16_t EE_status;


	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);

  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	BSP_LCD_GLASS_Clear();


******************************testing I2C EEPROM*****************************/	
		

  /* Infinite loop */
  while (1)
  {
			//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
			//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
			//therefore, the variable "selpressed==1" can not be used to make choice here.
			if (BSP_JOY_GetState() == JOY_SEL) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>1000) {
								RTC_DateShow(&RTCHandle);
						} 
					}
			}					
//==============================================================			

			if (downpressed==1) {
				if (state == TIMEDATESET)
					switch (settingpos) {
						case SECOND_POS:
							ss = (ss - 1)%60;
							break;
						case MINUTE_POS:
							mm = (mm - 1)%60;
							break;
						case HOUR_POS:
							hh = (hh - 1)%24;
							break;
						case DATE_POS:
							wd = (wd - 1)%7;
							break;
						case DAY_POS:
							dd = (dd - 1)%31+1;
							break;
						case MONTH_POS:
							mo = (mo - 1)%12+1;
							break;
						case YEAR_POS:
							yy = (yy - 1)%100;
							break;
					}
					
					sprintf(timestring,"%02u%02u%02u",hh,mm,ss); 
					sprintf(datestring,"%02u%02u%02u",dd,mo,yy);
					if (settingpos < 3) {
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
					}
					else if (settingpos >= 3 && settingpos < 7) {
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)datestring);
					}
					downpressed=0;
			}
//==============================================================					
			if (selpressed==1)  {
					state = PRESSRECORD;
					selpressed=0;
			} 
//==============================================================			

			if (uppressed==1) {
				if (state == TIMEDATESET)
					switch (settingpos) {
						case SECOND_POS:
							ss = (ss + 1)%60;
							break;
						case MINUTE_POS:
							mm = (mm + 1)%60;
							break;
						case HOUR_POS:						
							hh = (hh + 1)%24;
							break;
						case DATE_POS:
							wd = (wd + 1)%7;
							break;
						case DAY_POS:
							dd = (dd + 1)%31;
							break;
						case MONTH_POS:
							mo = (mo + 1)%12;
							break;
						case YEAR_POS:
							yy = (yy + 1)%100;
							break;
					}
					
					sprintf(timestring,"%02u%02u%02u",hh,mm,ss); 
					sprintf(datestring,"%02u%02u%02u",dd,mo,yy);
					if (settingpos < 3) {
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
					}
					else if (settingpos >= 3 && settingpos < 7) {
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)datestring);
					}
				uppressed=0;
			}
			
//==============================================================		 
			if (leftpressed==1) {
					if (state == PRESSSHOW)
						state = TIMESHOW;
					else if (state == TIMEDATESET) {
						settingpos = (settingpos + 1) %7;
					}
					else
						state = PRESSSHOW;
							
					leftpressed=0;
			}			
//==============================================================			

			if (rightpressed==1) {
				if (state == DATESHOW)
					state = TIMESHOW;
				else if (state == TIMEDATESET)
					settingpos = (settingpos - 1) % 7;
				else
					state = DATESHOW;
				
				rightpressed=0;
			}
//==============================================================							
			if (push1pressed==1) {
					if (state == TIMEDATESET) {
						state=TIMESHOW;
						RTC_Config();
						RTC_AlarmA_IT_Enable(&RTCHandle);
					}
					else
						state=TIMEDATESET;
			
					push1pressed=0;
			}
//==============================================================			

//==============================================================						
			switch (state) { 
				
				case TIMESHOW:
					break;
				
				case DATESHOW:
					RTC_DateShow(&RTCHandle);
					state = TIMESHOW;
					break;
				
				case PRESSRECORD:
					if (EE_RecordTime(&RTCHandle) != HAL_OK)
						Error_Handler();
					state=TIMESHOW;
					break;
					
				case PRESSSHOW:
					EE_DisplayTime(&RTCHandle);
					state=TIMESHOW;
					break;
				
				case TIMEDATESET:
					RTC_AlarmA_IT_Disable(&RTCHandle);
					break;
				
			} //end of switch					
		


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

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

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
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz


void Pushbutton1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority((IRQn_Type)EXTI15_10_IRQn, 3, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)EXTI15_10_IRQn);
	
}

void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		
		//************students: need to complete the following lines******************************
		//**************************************************************************************				
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
				
				RTCHandle.Init.AsynchPrediv = 127; 
				RTCHandle.Init.SynchPrediv = 255; 
				
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_ALARMA;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 
				}
				
	//******************************************************************************************
	
	
	//****3.***** init the time and date
				
				
 		//*****************Students: please complete the following lnes*****************************
		//****************************************************************************************		
				RTC_DateStructure.Year = yy;
				RTC_DateStructure.Month = mo;
				RTC_DateStructure.Date = dd;
				RTC_DateStructure.WeekDay = wd;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
  
				RTC_TimeStructure.Hours = hh;  
				RTC_TimeStructure.Minutes = mm; 
				RTC_TimeStructure.Seconds = ss;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
	  
 //********************************************************************************



				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to ‘1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

	//**************students:  you need to set the followint two lines****************
	//********************************************************************************
	
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;
	
	
	//********************************************************************************/			
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

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
						selpressed=1;	
						break;	
			case GPIO_PIN_1:     //left button						
							leftpressed=1;
							break;
			case GPIO_PIN_2:    //right button						  to play again.
							rightpressed=1;			
							break;
			case GPIO_PIN_3:    //up button						
							uppressed=1;
							break;		
			case GPIO_PIN_5:    //down button						
							downpressed=1;
							break;
			case GPIO_PIN_14:    //push button 1
							push1pressed=1;
							break;
			case GPIO_PIN_11:    //push button 2					
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"PE11");
							break;				
			default://
						//default
						break;
	  } 
}

HAL_StatusTypeDef EE_RecordTime(RTC_HandleTypeDef *hrtc)
{
	//RTC_AlarmA_IT_Disable(hrtc);
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure, RTC_FORMAT_BIN);
	
	ss=RTC_TimeStructure.Seconds;
	mm=RTC_TimeStructure.Minutes;
	hh=RTC_TimeStructure.Hours;
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, ss);
  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
	}
	
	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1, mm);
  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
	}
	
	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+2, hh);
  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
	}
	
	if (memLocation < 0xFFFF)
	{
		memLocation=memLocation+3;
		
		EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, 0x0000, memLocation);
  
		if(EE_status != HAL_OK)
		{
			I2C_Error(&pI2c_Handle);
		}
	
		HAL_Delay(1000);
	}
	else
	{
		memLocation=0x0001;
		EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, 0x0000, memLocation);
  
		if(EE_status != HAL_OK)
		{
			I2C_Error(&pI2c_Handle);
		}
	
		HAL_Delay(1000);
	}
	//RTC_AlarmA_IT_Enable(hrtc);
	
	return EE_status;
}

void EE_DisplayTime(RTC_HandleTypeDef *hrtc)
{
	RTC_AlarmA_IT_Disable(hrtc);
	
	if (memLocation < 0x06)
	{
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"TOOFEW");
	}
	else 
	{
		hh=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-1);
		HAL_Delay(1000);
		mm=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-2);
		HAL_Delay(1000);
		ss=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-3);
		
		sprintf(timestring,"%02u%02u%02u",hh,mm,ss);
		
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"time1");
		HAL_Delay(1000);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
		
		hh=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-4);
		HAL_Delay(1000);
		mm=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-5);
		HAL_Delay(1000);
		ss=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-6);
		
		sprintf(timestring,"%02u%02u%02u",hh,mm,ss);

		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"time2");
		HAL_Delay(1000);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
		HAL_Delay(1000);
		BSP_LCD_GLASS_Clear();
	}
	
	RTC_AlarmA_IT_Enable(hrtc);
}

void RTC_TimeShow(RTC_HandleTypeDef *hrtc)
{
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure, RTC_FORMAT_BIN);
	
	ss=RTC_TimeStructure.Seconds;
	mm=RTC_TimeStructure.Minutes;
	hh=RTC_TimeStructure.Hours;
	
	sprintf(timestring,"%02u%02u%02u",hh,mm,ss); 
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
}

void RTC_DateShow(RTC_HandleTypeDef *hrtc)
{
	RTC_AlarmA_IT_Disable(&RTCHandle);
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure, RTC_FORMAT_BIN);

	dd=RTC_DateStructure.Date;
	mo=RTC_DateStructure.Month;
	yy=RTC_DateStructure.Year;
	
	sprintf(datestring," d%02u m%02u y%02u ",dd,mo,yy);
	
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)datestring,1,500);
	RTC_AlarmA_IT_Enable(&RTCHandle);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  BSP_LED_Toggle(LED5);
	BSP_LCD_GLASS_Clear();
	RTC_TimeShow(hrtc);
	
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
