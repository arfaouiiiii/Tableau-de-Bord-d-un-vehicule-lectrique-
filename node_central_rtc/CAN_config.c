#include "CAN_config.h"
#include "stm32f4xx_can.h"
#include "RTC_config.h"
#include "LCD_config.h"
#include "ESP_config.h"

char buffer3[30];
char buffer4[30];
int f = 0;
char conv[10];
int channel;
uint32_t tableau[4];
CanTxMsg TxMessage;
char buffer[4];
CanRxMsg RxMessage;
CanRxMsg messageBuffer[4];  // Buffer circulaire pour stocker les messages
volatile uint8_t Index = 0;     // Index d'?criture dans le buffer
 

void config_CAN(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // Activer l'horloge GPIOD
	GPIOD->MODER |= (0x1 << 28);  // Configurer PD14 en mode sortie
	GPIOD->MODER |= (0x1 << 30);  // Configurer PD15 en mode sortie
	GPIOD->MODER |= (0x1 << 26);  // Configurer PD14 en mode sortie
	GPIOD->MODER |= (0x1 << 24);  // Configurer PD15 en mode sortie

	RCC->APB1ENR |= 1 << 25; // Activer l'horloge pour CAN1

	RCC->AHB1ENR |= 1 << 1;  // Activation de l'horloge pour GPIOB
	GPIOB->MODER |= (2 << 16);  // Alternate Function pour PB8 (CAN RX)
	GPIOB->MODER |= (2 << 18);  // Alternate Function pour PB9 (CAN TX)
	GPIOB->AFR[1] |= (9 << 0) | (9 << 4); // AF9 (CAN) pour PB8 et PB9

	CAN_InitTypeDef CAN_InitStructure;
	// Initialiser les param?tres CAN
	CAN_InitStructure.CAN_Prescaler = 2;  // Ajuste la vitesse du bus CAN
	CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;  // Mode Loopback activ?
	CAN_InitStructure.CAN_BS1 = 10;  // Bit Segment 1 (nombre de TQ)
	CAN_InitStructure.CAN_BS2 = 5;  // Bit Segment 2 (nombre de TQ)
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  // Time Quantum de resynchronisation
	 // Initialiser CAN avec les param?tres d?finis
	CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;                  // Utiliser le filtre 0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // Mode masque
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // Filtre 32 bits
	CAN_FilterInitStructure.CAN_FilterIdHigh = (0x680 << 5);       // ID de filtre 0x680 (standard)
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;              // Bas du filtre
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (0x7FC << 5);   // Masque d'ID 0x7FC (standard)
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;          // Masque bas
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;  // Assigner au FIFO 0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;          // Activer le filtre
	CAN_FilterInit(&CAN_FilterInitStructure); // Appliquer la configuration du filtre

	CAN1->IER |= CAN_IER_TMEIE; // Interruption pour message transmis
	NVIC_EnableIRQ(CAN1_TX_IRQn);

	// Activer l'interruption pour r?ception
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN1->IER |= CAN_IER_FMPIE0;
	// Configurer l'interruption dans le NVIC
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void hse_clk(void) // 8MHz
{
	RCC->CFGR = 0x01; //select HSE as system clock
	RCC->CR |= 1<<16; //HSE on: activation du HSE par le bit 16
	while (!(RCC->CR & (1<<17))); //wait until HSE ready
}


void Send_CAN_Message(char *message, uint16_t stdId )
{
    uint8_t i = 0;
    uint8_t j = 0;
    TxMessage.StdId = stdId;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 3 ;

    for (j = 0; j < TxMessage.DLC; j++)
      {
        TxMessage.Data[j] = message[i++];
      }

     CAN_Transmit(CAN1, &TxMessage);


}

void delay_ms(uint32_t ms)
{
	uint32_t i ;
    for (i = 0; i < ms * 1000; i++)
    {

    }
}

void prototype(void)
{
     delay_ms(1000);
	
	   if (f == 0)
		 {
		 f ++;
     sprintf(buffer,"Ch1");   // Formatez la cha?ne pour le canal 1
	   Send_CAN_Message(buffer, 0x680);
		 GPIOD->ODR = 0x8000;  // Turn on led pd14	
     delay_ms(500);			 
     }
		 
     else if ( f == 1 )//CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
		 {	
                CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);													    
                if (RxMessage.StdId == 0x680)
                       {
												 f++;
                         // Stocker le message dans le buffer circulaire
	        	             messageBuffer[Index] = RxMessage;
	        	             Index ++; 	
												 //RxMessage.StdId = 0;
                         sprintf(buffer,"Ch2");   // Formatez la cha?ne pour le canal 1
	       	               Send_CAN_Message(buffer, 0x681);
				                 GPIOD->ODR = 0xC000;  // Turn on led pd14
                         delay_ms(500);                        												 
                       }
			}
											 
			else if (f == 2)
			{				
				          CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);	
                  if (RxMessage.StdId == 0x681)
                      {
                       f ++;												
                            // Stocker le message dans le buffer circulaire
	        	           messageBuffer[Index] = RxMessage;
	        	           Index ++;													
	        	           sprintf(buffer,"Ch3");   // Formatez la cha?ne pour le canal 1
	        	           Send_CAN_Message(buffer, 0x682);
				               GPIOD->ODR = 0xE000;  // Turn on led pd14											 
                      }
			}
			
			else if (f == 3)
			{
				         CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);	
                 if (RxMessage.StdId == 0x682)
						           {
                       f ++;												 
	        	          // Stocker le message dans le buffer circulaire
	        	           messageBuffer[Index] = RxMessage;
	        	           Index ++;
                       sprintf(buffer,"Ch4");   // Formatez la cha?ne pour le canal 1
	        	           Send_CAN_Message(buffer, 0x683);												 
                       }
			}
			else if (f == 4)
			{
				CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
				if (RxMessage.StdId == 0x683)
				{
              f ++;												 
	      // Stocker le message dans le buffer circulaire
	        	 messageBuffer[Index] = RxMessage;
	        	 Index ++;
					  GPIOD->ODR = 0x8000;
					f = 0;
	}

}		
			
}

/*void affichage(void)
{
	 sprintf(buffer3, "ch1=%dch2=%d" ,tableau[0] ,tableau[1]);     
   LCD_Send_String(buffer3); // Display first line
   LCD_Send_Command(0xC0); // Move cursor to the second line
	 sprintf(buffer4, "ch3=%dtemp=%d" ,tableau[2] ,tableau[3] );
   LCD_Send_String(buffer4); // Display second line
	
}
*/

/*void transmission(void)
{
	 Send_Data_To_ThingSpeak(tableau[3], tableau[0], tableau[1], tableau[2]);
	 delay_ms(3000);
}
*/


/*void conversion(void)
{
for (int i = 0; i < 4; i++)
	   {
			 strncpy(conv, (char *)messageBuffer[i].Data, sizeof(messageBuffer[i].Data));
			 sscanf(conv, "Ch%d=%d", &channel, &tableau[i]);			 
		 }
}
*/
/*void write_in_backup(void)
{
    BackupSRAM_Write(0, tableau[0]); // Write 100 at offset 0
    BackupSRAM_Write(1, tableau[1]); // Write 200 at offset 1
    BackupSRAM_Write(2, tableau[2]); // Write 300 at offset 2
	  BackupSRAM_Write(3, tableau[3]); // Write 300 at offset 2
}
*/
void CAN1_TX_IRQHandler(void)
{
    if (CAN1->TSR & CAN_TSR_RQCP0)
    {
        CAN1->TSR |= CAN_TSR_RQCP0; // Effacer le flag de requ?te


    }
}


void CAN1_RX0_IRQHandler(void)
{
	prototype();

}

