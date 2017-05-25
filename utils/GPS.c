#include "GPS.h"

//uint16_t Long_Deg = 0;
//float Long_Min = 0;
//uint16_t Lat_Deg = 0;
//float Lat_Min = 0;
//uint16_t altitude = 0;
uint8_t fix = 0;
uint8_t heading = 0;

uint8_t parse_GPGGA(uint8_t* sentence[], uint16_t* altitude, uint16_t* Long_Deg, float* Long_Min, uint16_t* Lat_Deg, float* Lat_Min)
{
	uint8_t a=0;
	uint8_t state = 3+'0';
	uint8_t pointer=0;
	//while(sentence[pointer]!='$')
	//{
	//	pointer++;
	//}
	if(sentence[1]=='G')
	{
		//USART1_Send(NEMA_String[1]);
		if(sentence[2]=='P')
		{
			//USART1_Send(NEMA_String[2]);
			if(sentence[3]=='G')
			{
				//USART1_Send(NEMA_String[3]);
				if(sentence[4]=='G')
				{
					//USART1_Send(NEMA_String[4]);
					if(sentence[5]=='A')
					{
						//USART1_Send(NEMA_String[5]);
						pointer=0;
						//while((sentence[string_pointer] != '\n') && (sentence[string_pointer] != '\r'))
						//{
							//USART1_Send(sentence[string_pointer]);
							//string_pointer++;
						//}
						//USART1_Send('\r');
						//USART1_Send('\n');
						//USART3_rx=0;
						uint8_t no_commas=0;
						//GPS_TypeDef *fix_data;
						//fix_data = malloc(sizeof(GPS_TypeDef));
						for(pointer=0;pointer!=69;pointer++)
						{
							if(sentence[pointer] == ',')
							{
								no_commas++;
							}
							if ((no_commas == 6) && (a!=5))
							{
								a=5;
								uint8_t temp_pointer = pointer;
								temp_pointer++;
								fix = (sentence[temp_pointer] - '0');
								if (fix == 0)
								{
									state = fix+'0';
									break;
								}else if(fix == 1)
								{
									state = 1+'0';
								}
							}
						}
						no_commas=0;
						if(fix != '0')
						{
						for(pointer=0;pointer!=69;pointer++)
						{
							if(sentence[pointer] == ',')
							{
								no_commas++;
							}
							if((no_commas==2) && (a!=1))
							{
								a=1;
								uint8_t temp_pointer = pointer;
								uint16_t temp=0;
								float temp2=0;
								while(sentence[temp_pointer] != '.')
								{
									temp_pointer++;
								}
								temp_pointer=temp_pointer-3;
								//string_pointer = string_pointer+3;
								*Lat_Deg = (sentence[temp_pointer] - '0');
								temp_pointer--;
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*10;
								*Lat_Deg = *Lat_Deg + temp;
								temp_pointer--;
								temp=0;
								}
								else{temp_pointer=temp_pointer+2;}
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*100;
								*Lat_Deg = *Lat_Deg + temp;
								}
								//Display_Longitude(fix_data.Lat_Deg);
								while(sentence[temp_pointer] != '.')
								{
									temp_pointer++;
								}
								temp_pointer=temp_pointer-2;
								temp = (sentence[temp_pointer] - '0');
								*Lat_Min = temp*10;
								temp_pointer++;
								temp = (sentence[temp_pointer] - '0');
								*Lat_Min = *Lat_Min + temp;
								temp_pointer=temp_pointer+2;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/10;
								*Lat_Min = *Lat_Min + temp2;
								temp_pointer++;
								temp=0;
								temp2=0;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/100;
								*Lat_Min = *Lat_Min + temp2;
								temp_pointer++;
								temp=0;
								temp2=0;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/1000;
								*Lat_Min = *Lat_Min + temp2;
								temp_pointer++;
								temp=0;
								temp2=0;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/10000;
								*Lat_Min = *Lat_Min + temp2;
								temp=0;
								temp2=0;
							}
							else if((no_commas==4) && (a!=3))
							{
								a=3;
								uint8_t temp_pointer = pointer;
								uint16_t temp=0;
								float temp2 = 0;
								while(sentence[temp_pointer] != '.')
								{
									temp_pointer++;
								}
								temp_pointer=temp_pointer-3;
								*Long_Deg = (sentence[temp_pointer] - '0');
								temp_pointer--;
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*10;
								*Long_Deg = *Long_Deg + temp;
								temp_pointer--;
								temp=0;
								}
								else{temp_pointer=temp_pointer+2;}
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*100;
								*Long_Deg = *Long_Deg + temp;
								}
								//Display_Longitude(fix_data.Long_Deg);
								while(sentence[temp_pointer] != '.')
								{
									temp_pointer++;
								}
								temp_pointer=temp_pointer-2;
								temp = (sentence[temp_pointer] - '0');
								*Long_Min = temp*10;
								temp_pointer++;
								temp = (sentence[temp_pointer] - '0');
								*Long_Min = *Long_Min + temp;
								temp_pointer=temp_pointer+2;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/10;
								*Long_Min = *Long_Min + temp2;
								temp_pointer++;
								temp=0;
								temp2=0;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/100;
								*Long_Min = *Long_Min + temp2;
								temp_pointer++;
								temp=0;
								temp2=0;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/1000;
								*Long_Min = *Long_Min + temp2;
								temp_pointer++;
								temp=0;
								temp2=0;
								temp = (sentence[temp_pointer] - '0');
								temp2 = (float)temp;
								temp2 = temp2/10000;
								*Long_Min = *Long_Min + temp2;
								temp=0;
								temp2=0;
							}
							else if((no_commas==9) && (a!=8))
							{
								a=8;
								uint8_t temp_pointer = pointer;
								uint16_t temp=0;
								while(sentence[temp_pointer] != '.')
								{
									temp_pointer++;
								}
								temp_pointer++;
								*altitude = (sentence[temp_pointer] - '0');
								temp_pointer=temp_pointer-2;
								temp = (sentence[temp_pointer] - '0');
								temp = temp*10;
								*altitude = *altitude + temp;
								temp_pointer--;
								temp=0;
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*100;
								*altitude = *altitude + temp;
								temp_pointer--;
								temp=0;
								}
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*1000;
								*altitude = *altitude + temp;
								temp_pointer--;
								temp=0;
								}
								if(sentence[temp_pointer] != ',')
								{
								temp = (sentence[temp_pointer] - '0');
								temp = temp*10000;
								*altitude = *altitude + temp;
								temp_pointer--;
								temp=0;
								}
							}
						}
						}
					}else {state = sentence[5];}
				}else {state = sentence[4];}
			}else {state = sentence[3];}
		}else {state = sentence[2];}
	}else {state = sentence[1];}
	return state;
}

uint8_t parse_GPHDT(uint8_t sentence[])
{
	uint8_t state = 1;
	uint8_t pointer=0;
	if(sentence[1]=='G')
	{
		if(sentence[2]=='P')
		{
			if(sentence[3]=='H')
			{
				if(sentence[4]=='D')
				{
					if(sentence[5]=='T')
					{
						state = 0;
						pointer=0;
						//GPS_TypeDef fix_data;
						for(pointer=0;pointer!=20;pointer++)
						{
							if(sentence[pointer] == '.')
							{
								uint8_t a = 1;
								uint8_t temp = 0;
								while(sentence[pointer] != ",")
								{
									temp = (sentence[pointer] - '0');
									temp=temp*a;
									heading = heading + temp;
									a=a*10;
									pointer=pointer-1;
								}


							}
						}
					}else {state = 2;}
				}else {state = 2;}
			}else {state = 2;}
		}else {state = 2;}
	}else {state = 2;}
	return state;
}
