#include <stdbool.h>
#define Vdd 3
#define TwelveBits 4095
#define Size 33
#define Timeout 10
#define I2C_ADDRESS   0x40
/* Size of Transmission buffer */
#define TXBUFFERSIZE                      20
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
#define EXPERIMENTSIZE  sizeof(experiments)
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


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
}experiment_package;

void Test_SiC(void);
void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength);
void send_message(uint8_t aTxBuffer[],uint16_t BUFFERSIZE);
void receive_message(void);
void User_Button(void);
