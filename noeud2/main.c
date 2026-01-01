#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

CanTxMsg CAN1_TxMsg;
CanRxMsg CAN1_RxMsg;

void CAN1_GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3<<(8*2))|(3<<(9*2)));
    GPIOB->MODER |= ((2<<(8*2))|(2<<(9*2)));
    GPIOB->AFR[1] &= ~((0xF<<0)|(0xF<<4));
    GPIOB->AFR[1] |= ((9<<0)|(9<<4));
}

void CAN1_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    CAN_DeInit(CAN1);
    CAN_InitTypeDef CAN_Config;
    CAN_StructInit(&CAN_Config);

    CAN_Config.CAN_Mode = CAN_Mode_Normal;
    CAN_Config.CAN_SJW = CAN_SJW_1tq;
    CAN_Config.CAN_BS1 = CAN_BS1_13tq;
    CAN_Config.CAN_BS2 = CAN_BS2_2tq;
    CAN_Config.CAN_Prescaler = 6;
    CAN_Init(CAN1, &CAN_Config);

    CAN_FilterInitTypeDef CAN_FilterConfig;
    CAN_FilterConfig.CAN_FilterNumber = 0;
    CAN_FilterConfig.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterConfig.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterConfig.CAN_FilterIdHigh = 0x0000;
    CAN_FilterConfig.CAN_FilterIdLow = 0x0000;
    CAN_FilterConfig.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterConfig.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterConfig);
}

void CAN1_Send_OK(void)
{
    CAN1_TxMsg.StdId = 0x22;
    CAN1_TxMsg.IDE = CAN_Id_Standard;
    CAN1_TxMsg.RTR = CAN_RTR_Data;
    CAN1_TxMsg.DLC = 2;
    CAN1_TxMsg.Data[0] = 'O';
    CAN1_TxMsg.Data[1] = 'K';

    uint8_t mb = CAN_Transmit(CAN1, &CAN1_TxMsg);
    while(CAN_TransmitStatus(CAN1, mb) != CAN_TxStatus_Ok);
}

int main(void)
{
    CAN1_GPIO_Init();
    CAN1_Config();
while(1)
{
    // 1. envoyer la demande
    CAN1_TxMsg.StdId = 0x33;
    CAN1_TxMsg.IDE = CAN_Id_Standard;
    CAN1_TxMsg.RTR = CAN_RTR_Data;
    CAN1_TxMsg.DLC = 1;
    CAN1_TxMsg.Data[0] = 0x00; // donnée arbitraire
    uint8_t mb = CAN_Transmit(CAN1, &CAN1_TxMsg);
    while(CAN_TransmitStatus(CAN1, mb) != CAN_TxStatus_Ok);

    // 2. attendre la réponse
    while(!CAN_MessagePending(CAN1, CAN_FIFO0));
    CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RxMsg);
    if(CAN1_RxMsg.StdId == 0x11)
    {
        uint16_t adc0 = (CAN1_RxMsg.Data[1]<<8)|CAN1_RxMsg.Data[0];
        uint16_t adc1 = (CAN1_RxMsg.Data[3]<<8)|CAN1_RxMsg.Data[2];
        uint16_t adc2 = (CAN1_RxMsg.Data[5]<<8)|CAN1_RxMsg.Data[4];
        // traitement ADC
    }

    for(volatile int i=0; i<1000000; i++); // délai entre demandes
}

}
