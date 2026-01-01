#include "stm32f4xx.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"



void USART2_Init(void);
void Send_String(char *str) ;
void Wait_Response(void);
void ESP8266_Config(void) ;
void Send_Data_To_ThingSpeak(float temp, int adc1, int adc2, int adc3) ;
