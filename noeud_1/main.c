#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

uint16_t ADC_Values[3] = {0, 0, 0};
char buff[3] ;
char buffer[9];
CanTxMsg TxMessage;
CanRxMsg RxMessage;
int k;



void config_CAN(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // Activer l'horloge GPIOD
	GPIOD->MODER |= (0x1 << 28);  // Configurer PD14 en mode sortie
	GPIOD->MODER |= (0x1 << 30);  // Configurer PD14 en mode sortie
	GPIOD->MODER |= (0x1 << 26);  // Configurer PD14 en mode sortie
	GPIOD->MODER |= (0x1 << 24);  // Configurer PD14 en mode sortie

	RCC->APB1ENR |= 1 << 25; // Activer l'horloge pour CAN1

	RCC->AHB1ENR |= 1 << 1;  // Activation de l'horloge pour GPIOB
	GPIOB->MODER |= (2 << 16);  // Alternate Function pour PB8 (CAN RX)
	GPIOB->MODER |= (2 << 18);  // Alternate Function pour PB9 (CAN TX)
	GPIOB->AFR[1] |= (9 << 0) | (9 << 4); // AF9 (CAN) pour PB8 et PB9

	CAN_InitTypeDef CAN_InitStructure;
	// Initialiser les param?tres CAN
	CAN_InitStructure.CAN_Prescaler = 2;  // Ajuste la vitesse du bus CAN
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  // Mode Loopback activ?
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

void Config_TIMER2(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 7999; // Set prescaler to 7999 to get Ti=1ms
    TIM2->ARR = 999;  // Set auto-reload to 999 to get T_cycle = 1s
    TIM2->DIER |= TIM_DIER_UDE; // Enable Update DMA request
    TIM2->CR2 |= TIM_CR2_MMS_1; // Master mode selection Update trigger output
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable Timer 2 interrupt
    TIM2->CR1 |= TIM_CR1_CEN;   // Enable TIM2
}


void Config_ADC1(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC1 clock enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA clock enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOC clock enable
    GPIOA->MODER |= GPIO_MODER_MODER4;   // PA4 (CH4): analog mode
    GPIOC->MODER |= GPIO_MODER_MODER4;   // PC4 (CH14): analog mode
    GPIOC->MODER |= GPIO_MODER_MODER5;   // PC5 (CH15): analog mode

    ADC1->CR1 |= ADC_CR1_SCAN;           // Scan mode (multiple channels)
    ADC1->CR1 &= ~ADC_CR1_DISCEN;        // Disable discontinuous mode
    ADC1->CR1 |= ADC_CR1_EOCIE;          // Enable EOC interrupt
    ADC1->SQR1 |= ADC_SQR1_L_1;          // 3 conversions (L = 2 means 3 conversions)
    ADC1->SQR3 |= (0x4 << 0) | (0xE << 5) | (0xF << 10); // Conversion order: CH4 (PA4), CH14 (PC4), CH15 (PC5)

    // Set the sample time for each channel (minimum is 3 cycles, can be increased)
    ADC1->SMPR2 |= (0x3 << (3 * 4)) | (0x3 << (3 * 14)) | (0x3 << (3 * 15)); // Sample time for better accuracy

    ADC1->CR2 |= ADC_CR2_ADON;           // Enable ADC1
    ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA; // Enable DMA and DDS (DMA continues after each conversion)
    ADC1->CR2 |= ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2; // Trigger with Timer 2
    ADC1->CR2 |= ADC_CR2_EXTEN_0; //Trigger Detection Rising Edge
    ADC1->CR2 |= ADC_CR2_DMA;   //Mode DMA
    ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2; //Timer 2 TRGO event
}

void Config_DMA2(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;  // DMA2 clock enable

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;    // Disable DMA Stream to configure it
    DMA2_Stream0->CR &= ~(DMA_SxCR_CHSEL); // Select Channel 0
    DMA2_Stream0->CR &= ~(DMA_SxCR_PL_0 | DMA_SxCR_PL_1); //Priority level low
    DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0; //Memory data size 16 bits
    DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0; //Peripheral data size 16 bits
    DMA2_Stream0->CR &= ~(DMA_SxCR_DIR_0 | DMA_SxCR_DIR_1); //Peripheral to Memory
    DMA2_Stream0->CR |= DMA_SxCR_MINC; //Memory increment mode enable
    DMA2_Stream0->CR |= DMA_SxCR_CIRC | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC; // Circular, 16-bit size
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;   // Transfer complete interrupt enable
    DMA2_Stream0->NDTR = 3;              // 3 data to transfer (for 3 ADC channels)
    DMA2_Stream0->M0AR = (uint32_t)&ADC_Values[0]; // Destination address in memory
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR; // Source address (ADC data register)
    DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS;  // Enable direct mode
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);   // Enable DMA interrupt
    DMA2_Stream0->CR |= DMA_SxCR_EN;     // Finally, enable DMA Stream
}
void Send_CAN_Message(char *message, uint16_t stdId )
{
    uint8_t i = 0;
    uint8_t j = 0;
    TxMessage.StdId = stdId;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8 ;

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
          if (CAN_MessagePending(CAN1 , CAN_FIFO0) > 0)
					{						
	        	        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);					  
					          if (RxMessage.StdId == 0x680)
										{
											//RxMessage.StdId = 0;
                      sprintf(buffer,"Ch1=%d",ADC_Values[0]);   // Formatez la cha?ne pour le canal 1
	        	          Send_CAN_Message(buffer, 0x680);
	        	          GPIOD->ODR = 0x8000;  // Turn on led pd14		
	        	        }
										else if (RxMessage.StdId == 0x681)
	        	        {	
                      //RxMessage.StdId = 0;											
	        	          sprintf(buffer,"Ch2=%d",ADC_Values[1]);
	        	          Send_CAN_Message(buffer, 0x681);
	        	          GPIOD->ODR = 0xC000;  // Turn on led pd14	                      												
						        } 
	        	        else if (RxMessage.StdId == 0x682)
	        	        {		
											//RxMessage.StdId = 0;
	        	        	sprintf(buffer,"Ch3=%d",ADC_Values[2]);
	        	          Send_CAN_Message(buffer, 0x682);
	        	          GPIOD->ODR = 0xE000;  // Turn on led pd14
                      											
	        	        } 	     
									}
}

int main(void)
{
	hse_clk();
	Config_TIMER2();
  Config_ADC1();
  Config_DMA2();
  config_CAN();	
	//GPIOD->ODR = 0x1000;
	
		
    while (1)
    {
			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; 
    __WFI(); 
    }
}


void TIM2_IRQHandler(void)
{
    if((TIM2->SR & TIM_SR_UIF) != 0)
    {   // Check if update flag is set
        TIM2->SR &= ~TIM_SR_UIF;         // Clear update flag
    }
}

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
    }
}



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

















