/**
  ******************************************************************************
  * File Name          : main.c
	
	Main program for SiC in Space experiment. 
	
	This is a test progam that will test the functionality of SiC.
	
	
	Refer to the Diptrace files for schematics and PCB design for 
	further pin connections. 
	
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"
#include "headers.h"

/* Private variables ---------------------------------------------------------*/
/* External variables*/
extern ADC_ChannelConfTypeDef        sConfigAdc;
extern DAC_ChannelConfTypeDef 			 sConfigDac;
extern ADC_HandleTypeDef             hadc;
extern DAC_HandleTypeDef    				 hdac;
extern I2C_HandleTypeDef 							hi2c1;
experiment_package  	 experiments[8];
uint8_t aRxBuffer[RXBUFFERSIZE];


/*
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA2     ------> ADC_IN2
    PA3     ------> ADC_IN3
    PA5     ------> ADC_IN5
    PA6     ------> ADC_IN6
    PA7     ------> ADC_IN7
    PB0     ------> ADC_IN8
    PB1     ------> ADC_IN9 
		*/


int main(void)
{
	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();


	while(1)
		{
		Flush_Buffer((uint8_t *)aRxBuffer,RXBUFFERSIZE);	  //Flush the buffer so the OBC can send message
		receive_OBC_message();															//Receive instructions from the OBC
		check_OBC_message();																//Check instructions from the OBC
			
	
	}
		
}
/*Start experiment run and performe 4 readings and send them to the OBC*/
void StartExperiments()
	{					
  
	setDAC(0);
	HAL_Delay(2);
		
	/* Set DAC at voltage level 1 (3.1v 0xF07)*/
	setDAC(3.1);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){ 
		readRollingADC(0); //All inputs 16 times.
	}		
	setDAC(0);
	HAL_Delay(2);
		/* Set DAC at voltage level 1 (2.1v 0xA2E)*/
	setDAC(2.1);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){
		readRollingADC(1);
	}

		setDAC(0);
	HAL_Delay(2);
		/* Set DAC at voltage level 1 (1.1v 0x555)*/
	setDAC(1.1);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){
		
		readRollingADC(2);
	}
	setDAC(0);
	HAL_Delay(2);
		/* Set DAC at voltage level 1 (0.5v 0x260)*/
	setDAC(0.5);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++)			
	{
		
		readRollingADC(3);
	}
	/*This value are used to test the I2C communication*/
	/*	experiments[0].temperature =1567; // Value for 0 celsius
		experiments[0].ube = 3.1;
		experiments[0].vrb = 4.2;
		experiments[0].vrc = 5.3;
	
		experiments[1].temperature = 1486;  //Value for 10 Celsius
		experiments[1].ube = 7;
		experiments[1].vrb = 8;
		experiments[1].vrc = 9;
	
		experiments[2].temperature = 737;  // Value for 100 Celcius
		experiments[2].ube = 11;
		experiments[2].vrb = 12;
		experiments[2].vrc = 13;
	
	experiments[3].temperature = 2000;  // Value out of range
		experiments[3].ube = 11;
		experiments[3].vrb = 12;
		experiments[3].vrc = 13;
	*/
	for(int i = 0; i < 8; i++)
  {
  experiments[i].temperature = Convert_temperature(experiments[i].temperature);		//Convert read Voltage to temperature
  }

	/* Send message */
	send_message((uint8_t *)experiments);
 }
	
 /*Convert read voltage from the sensors to Temperature in Celsius*/
uint16_t Convert_temperature(uint16_t Vout)
{
	if((Vout >= 1956) || (Vout <= 660))			//If read temperature is less than -50 or higher than 150 degress return out of range
		return 65535;
	else
		{
  //uint16_t temp = (Vout - 1567)/(-7.76);
  uint16_t temp  =  (8.194 - pow((pow(-8.194,2) + 4*0.00262*(1324 - Vout)),0.5))/(2*(-0.00262)) + 30;
  return temp;
		}
}
void receive_OBC_message(){
	
	  /*##-2- Put I2C peripheral in reception process ###########################*/  
  while(HAL_I2C_Slave_Receive(&hi2c1, (uint8_t *)aRxBuffer, RXBUFFERSIZE,100) != HAL_OK)
  {
		
    /*If some instructions has been send then return to main*/
   if(aRxBuffer[0] != '\0')
		return;     
  }
	
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }
		return;
}	
 


void setDAC(double voltage){
	uint32_t Vdac = voltage*TwelveBits/Vdd;
  HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, Vdac);
  HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
}
void readRollingADC(int index){
	
	//Calibrate ADCs in the beginning of every run
	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK){
		Error_Handler();
	}

	HAL_Delay(1);

	//Start ADC reading
	if(HAL_ADC_Start(&hadc) != HAL_OK){
		while(1) {
			Error_Handler();
		}
	}
	
	//Wait for EOC (end of conversion)
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	//Read ADC value
	experiments[0+index].temperature += hadc.Instance->DR;

		
	//Repeat for all channels.	
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[0+index].ube += hadc.Instance->DR;
	
  while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[0+index].vrb += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[0+index].vrc += hadc.Instance->DR;
	
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].temperature += hadc.Instance->DR;

	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].ube += hadc.Instance->DR;
	
  while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].vrb += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].vrc += hadc.Instance->DR;
		
}

void send_message(uint8_t * message){
	
	/*************************DISCLAIMER****************
	This code for the I2C communication is not valid for communication and only
	used for test purposes.
	This code should be rewritten before use.
	********************************************************/
	if(HAL_I2C_Slave_Transmit(&hi2c1,message, (uint16_t)EXPERIMENTSIZE, 10000)!= HAL_OK)
	{
	  Error_Handler();
  
 
	}
	
	 return;
	
}
void SHDNLinearRegulator(bool a)
{
  if(a)
  { 
    HAL_GPIO_WritePin(SHDNLinearRegulator_GPIO_Port, SHDNLinearRegulator_Pin, GPIO_PIN_RESET);
 }
  else
  {
    HAL_GPIO_WritePin(SHDNLinearRegulator_GPIO_Port, SHDNLinearRegulator_Pin, GPIO_PIN_SET);
   
  }
  return;
}

void SHDNSwitchRegulator(bool a)
{
  if(a)
  { 
    HAL_GPIO_WritePin(SHDNSwitchRegulator_GPIO_Port,SHDNSwitchRegulator_Pin, GPIO_PIN_RESET);
   }
  else
  {
    HAL_GPIO_WritePin(SHDNSwitchRegulator_GPIO_Port,SHDNSwitchRegulator_Pin, GPIO_PIN_SET);
  
  }
  return;
}
void SHDNBatteryPower(bool a)
{
  if(a)
  { 
    HAL_GPIO_WritePin(BatteryPowerBus_GPIO_Port, BatteryPowerBus_Pin, GPIO_PIN_RESET);
   }
  else
  {
    HAL_GPIO_WritePin(BatteryPowerBus_GPIO_Port, BatteryPowerBus_Pin, GPIO_PIN_SET);
  
  }
  return;
}
void SHDNPowerPiezo5V(bool a)

{
  if(a)
  { 
    HAL_GPIO_WritePin(PowerPiezo5V_GPIO_Port, PowerPiezo5V_Pin, GPIO_PIN_RESET);
   }
  else
  {
    HAL_GPIO_WritePin(PowerPiezo5V_GPIO_Port, PowerPiezo5V_Pin, GPIO_PIN_SET);
;
  }
  return;
}


/** System Clock Configuration
Code auto generated by CubeMX
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_3;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void SystemPower_Config(void)
{
	//Used before entering power STOP mode.
	//Should be revised, could be optimized. 

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select HSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
  
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

}

void Error_Handler(void){

	HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_SET);
	return;

}
void check_OBC_message(void)
{
	
	if(strcmp(aRxBuffer, "SHDNLinear") == 0)
	{
		SHDNLinearRegulator(true);
		return;
	}
		if(strcmp(aRxBuffer, "SHDNSwitch") == 0)
	{
		SHDNSwitchRegulator(true);
		return;
	}
		if(strcmp(aRxBuffer, "SHDNPowerPiezo5V") == 0)
	{
		SHDNPowerPiezo5V(true);
		return;
	}
	if(strcmp(aRxBuffer, "SHDNBatteryPower") == 0)
	{
		SHDNBatteryPower(true);
		return;
	}
			if(strcmp(aRxBuffer, "StartExperiment") == 0)
	{
		SHDNBatteryPower(false);
		SHDNLinearRegulator(false);
		SHDNSwitchRegulator(false);
		StartExperiments();
		return;
	}
	else return;
}
static void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;

    pBuffer++;
  }
}
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
