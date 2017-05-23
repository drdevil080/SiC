#include <stdint.h>	//uint types
#include <stdlib.h>	//malloc and free


/*
if instead defined as:
typedef struct{
 ... 
 ... 
 }experiment_package;
 then keyword struct wont be necessary in variable declaration
*/
struct experiment_package {
	/*
	temperature
	Vrb (voltage over resistor on base)
	Vrc (voltage over resistor on collector)
	Ube (voltage frop from base to emitter)
	*/
	uint16_t temperature;
	uint16_t vrb;
	uint16_t vrc;
	uint16_t ube;
	
};


/*note call by reference to avoid copy overhead for struct
or just copy by removíng * from signature
callee is responsible for freeing memory after use

call to malloc shoudl use sizeof but sizeof wont work for reasons unknown

also this is endian-dependent
*/
void uint8_t* struct2byteArr(struct experiment_package* src){
	const uint8_t arrSize=2;
	const uint8_t fields = 4;
	const uint8_t fieldSize=2;
	
	//if not called by reference change to &src
	uint8_t* srcAsArray=(uint8_t*)src;

	/*
	if sizeof operator starts working replace argument to malloc with:
	arrSize*sizeof(struct experiments_package)
	*/

	//cast to uint8_t* for c++ compliance
	uint8_t* retPtr=(uint8_t*)malloc(arrSize*fields*fieldSize/*hardcoded assumed byte size*/);
	
	//copy memory 
	for(int i=0;i<arrSize*fields*fieldSize;i++){
		retPtr[i]=srcAsArray[i];
	}
	return retPtr;
}

