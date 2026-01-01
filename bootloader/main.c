#include "stm32f4xx.h"


int ncount1=0;





void config_TIMER6(void)
{
	RCC->APB1ENR |= 0x10;            // Enable TIM6 clock
	TIM6->PSC = 15999;               // Set prescaler to 41999
	TIM6->ARR = 999;                 // Set auto-reload to 5999
	TIM6->DIER |= 0x0001;           // Enable interrupt on update event
	NVIC_EnableIRQ(TIM6_DAC_IRQn);  // Enable TIM6 IRQ
	TIM6->CR1= 0x0001;	// Enable TIM6 counter (CEN=1 =>CR1=0x0001) and (update disable UDIS=0=>CR1=0x0004)
	SCB->VTOR =0x08000000;
}

void config_GPIOD()
{
	RCC->AHB1ENR |= 0x00000008;     // Enable GPIOD clock
  GPIOD->MODER |= 0x55<<24;       //Bits 12..15 are output
}


int main()
{
	
	config_TIMER6();
	config_GPIOD();
	
  while(1);
}



void TIM6_DAC_IRQHandler(void)
{
    if((TIM6->SR&0x1)!= 0)// If update flag is set
    {
			TIM6->SR&=0x0; // Interrupt has been handled
    	
			ncount1++;
    	
			if(ncount1==1)
    	{
    	  GPIOD->ODR = 0x8000; // Set D15 high
    	}
    	if(ncount1==2)
    	{
    	  GPIOD->ODR = 0x4000; // Set D14 high

    	}
    	if(ncount1==3)
    	{
    	   GPIOD->ODR = 0x2000; // Set D13 high
    	}
    	if(ncount1==4)
    	{
    	   GPIOD->ODR = 0x1000; // Set D12 high
				 
    	}
			if(ncount1==5)
    	{
    	   GPIOD->ODR = 0xF000; // Set D12 high
				 ncount1=0;
    	}
    	
    	
    }


}