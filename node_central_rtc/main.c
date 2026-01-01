#include "CAN_config.h"
//#include "RTC_config.h"
//#include "LCD_config.h"
//#include "ESP_config.h"


int main(void)
{
	 //hse_clk();
	 //BackupSRAM_Init();
	 config_CAN();
   prototype();
	 GPIOD->ODR = 0x1000;
	 delay_ms(1000); // Attendre 3 secondes
	 //conversion();
	 //I2C1_Init();	
   //LCD_Init();
	 //affichage();
   //write_in_backup();
	 //GPIOD->ODR = 0x8000;
	 //USART2_Init();
	 //ESP8266_Config();
	 //transmission();
	 //delay_ms(3000); // Attendre 3 secondes
	 //RTC_config();
	 //__WFI(); // Met le microcontr?leur en attente jusqu'? un ?v?nement	
  
    while(1)
    {
    	
    }
}
