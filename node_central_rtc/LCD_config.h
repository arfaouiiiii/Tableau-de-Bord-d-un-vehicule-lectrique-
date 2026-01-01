#include "stm32f4xx_can.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"


#define LCD_ADDRESS 0x4E 

void delay(uint32_t ms);
void I2C1_Init(void);
void I2C1_Write(uint8_t address, uint8_t data);
void LCD_Init(void);
void LCD_Send_Command(uint8_t command);
void LCD_Send_String(char *str);