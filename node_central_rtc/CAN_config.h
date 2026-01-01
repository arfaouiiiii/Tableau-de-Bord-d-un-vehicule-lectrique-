#ifndef __CAN_CONFIG_H
#define __CAN_CONFIG_H


#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

void config_CAN(void);
void hse_clk(void);
void Send_CAN_Message(char *message, uint16_t stdId) ;
void delay_ms(uint32_t ms);
void prototype(void);
void conversion(void);
void affichage(void);
void write_in_backup(void);
void transmission(void);
#endif /* __CAN_CONFIG_H */