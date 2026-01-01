#include "ESP_config.h"

char BUFFER[512];
char BUFFER1[50];

void USART2_Init(void) 
{
    
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Horloge pour USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Horloge pour GPIOA
    // Configurer PA2 (TX) et PA3 (RX) en mode Alternate Function
    // GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // Clear MODER bits	
	  // Alternate Function
	  GPIOA->MODER |= (2 << 4)|(2 << 6);
    // AF7 pour USART2
    GPIOA->AFR[0] |= (7 << 8)| (7 << 12);
	  GPIOA->OSPEEDR |= (0x03 << 4) | (0x03 << 6); // High Speed pour PA2 et PA3

    USART2->BRR = 8000000 / 115200;  
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE ; // Activer USART, RX, TX
}

void Send_String(char *str) 
{
    while (*str) 
			{
        USART2->DR = *str++;
        while (!(USART2->SR & USART_SR_TC));  // Attendre la fin de la transmission
    }
}


void Wait_Response(void)
{
    for (volatile int i = 0; i < 3000000; i++); // Delay
}

// configuration du module ESP8266
void ESP8266_Config(void) 
{
    Send_String("AT\r\n");
    Wait_Response();

    Send_String("AT+RST\r\n");
    Wait_Response();

    Send_String("AT+CWMODE=1\r\n");
    Wait_Response();

    Send_String("AT+CWJAP=\"Taha\",\"tamtam19\"\r\n");
    Wait_Response();
}

// envoi des donn?es ? thingspeak
void Send_Data_To_ThingSpeak(float temp, int adc1, int adc2, int adc3) 
{
    Send_String("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");
    Wait_Response();
    sprintf(BUFFER, "GET /update?api_key=Y0PWRGYBXJPDR2TQ&field1=%f&field2=%d&field3=%d&field4=%d\r\n",
            temp, adc1, adc2, adc3);
    sprintf(BUFFER1, "AT+CIPSEND=%d\r\n", strlen(BUFFER));
    Send_String(BUFFER1);
    Wait_Response();
    Send_String(BUFFER);
    Wait_Response();
}