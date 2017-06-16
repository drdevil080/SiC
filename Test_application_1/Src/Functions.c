#include "stm32f3xx_hal.h"
#include "functions.h"
extern DAC_HandleTypeDef hdac;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;


experiment_package experiments[8];
uint8_t aRxBuffer[32];
uint8_t aTxBuffer[20];


void send_message(uint8_t aTxBuffer[],uint16_t BUFFERSIZE){
  
  /*************************DISCLAIMER****************
  This code for the I2C communication is not valid for communication and only
  used for test purposes.
  This code should be rewritten before use.
  ********************************************************/
  
  
  
  while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)I2C_ADDRESS,(uint8_t*)aTxBuffer, (uint16_t)BUFFERSIZE,100)!= HAL_OK)
  {
    /* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }
	  /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      {
      }
  
  return;
}
void receive_message(void)
{
  while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)I2C_ADDRESS,(uint8_t *)experiments,(uint16_t)EXPERIMENTSIZE,(uint32_t)10000)!= HAL_OK)
	{
          if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
              Error_Handler();
     
          if(experiments[2].temperature != '\0')
              break;
	
        }
  
  return;
  
}
void User_Button()
{
  while ( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 1 ); 
    while ( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0 ); 
    return;
}
void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;

    pBuffer++;
  }
}

/*This function will test all the functions on the MCU on the SiC PCB.*/
void Test_SiC(void)
{   
  Flush_Buffer((uint8_t *)aTxBuffer,TXBUFFERSIZE);
  strcat(aTxBuffer, "SHDNLinear");
  send_message(aTxBuffer,(uint16_t)TXBUFFERSIZE);
  
  // To start next test push the User button on the board 
  User_Button();
  
  Flush_Buffer((uint8_t *)experiments,(uint16_t)EXPERIMENTSIZE);
  Flush_Buffer((uint8_t *)aTxBuffer,TXBUFFERSIZE);
  strcat(aTxBuffer, "StartExperiment");
  send_message(aTxBuffer,(uint16_t)TXBUFFERSIZE);
  receive_message();
 
// To start next test push the User button on the board 
  User_Button();
   
  Flush_Buffer((uint8_t *)aTxBuffer,TXBUFFERSIZE);
  strcat(aTxBuffer, "SHDNSwitch");
  send_message(aTxBuffer,(uint16_t)TXBUFFERSIZE); 
  
// To start next test push the User button on the board 
  User_Button();
  
  Flush_Buffer((uint8_t *)aTxBuffer,TXBUFFERSIZE);
  strcat(aTxBuffer, "SHDNPowerPiezo5V");
  send_message(aTxBuffer,(uint16_t)TXBUFFERSIZE); 
  
// To start next test push the User button on the board 
  User_Button();
  
  Flush_Buffer((uint8_t *)aTxBuffer,TXBUFFERSIZE);
  strcat(aTxBuffer, "SHDNBatteryPower");
  send_message(aTxBuffer,(uint16_t)TXBUFFERSIZE); 
  
// To start next test push the User button on the board 
  User_Button();
  
  Flush_Buffer((uint8_t *)experiments,(uint16_t)EXPERIMENTSIZE);
  Flush_Buffer((uint8_t *)aTxBuffer,TXBUFFERSIZE);
  strcat(aTxBuffer, "StartExperiment");
  send_message(aTxBuffer,(uint16_t)TXBUFFERSIZE);
  receive_message();
  
  return;
  }

