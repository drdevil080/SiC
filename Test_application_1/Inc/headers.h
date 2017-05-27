/* Private function prototypes -----------------------------------------------*/
#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include <inttypes.h>
#include <math.h>

#define EXPERIMENTSIZE sizeof(experiments)
	

void SystemClock_Config(void);
void send_message(uint8_t *);
void setDAC(double voltage);
void readRollingADC(int);
void StartExperiments(void);
void SystemPower_Config(void);
void SHDNLinearRegulator(bool a);
void SHDNSwitchRegulator(bool a);
void SHDNBatteryPower(bool a);
void SHDNPowerPiezo5V(bool a);
void Error_Handler(void);
void check_OBC_message(void);
void receive_OBC_message(void);
static void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength);

uint16_t Convert_temperature(uint16_t Vout);

typedef struct 
{
	/*temperature
	Vrb (voltage over resistor on base)
	Vrc (voltage over resistor on collector)
	Ube (voltage frop from base to emitter)
	*/
	uint16_t temperature;
	float vrb;
	float vrc;
	float ube;	
}experiment_package ;


