#ifndef __GPS_H
#define __GPS_H


#include <float.h>
#include "stm32f30x.h"

void get_coordinates();



//typedef struct
//{
//	extern uint16_t Long_Deg;
//	extern float Long_Min;
//	extern uint16_t Lat_Deg;
//	extern float Lat_Min;
//	extern uint16_t altitude;
	extern uint8_t fix;
	extern uint8_t heading;
//}GPS_TypeDef;

//extern GPS_TypeDef* fix_data;
uint8_t parse_GPGGA(uint8_t* sentence[], uint16_t* altitude, uint16_t* Long_Deg, float* Long_Min, uint16_t* Lat_Deg, float* Lat_Min);
uint8_t parse_GPHDT(uint8_t sentence[]);

#endif /* __MAIN_H */
