#include "LCD_config.h"




// I2C initialization
void I2C1_Init(void) 
	{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 clock

    // PB6 (SCL) and PB7 (SDA) configuration
    GPIOB->MODER |= (2 << (6 * 2)) | (2 << (7 * 2)); 
    GPIOB->OTYPER |= (1 << 6) | (1 << 7); 
    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4)); 
    GPIOB->PUPDR |= (1 << (6 * 2)) | (1 << (7 * 2)); 

    //I2C1 configuration
    I2C1->CR1 = 0;
    I2C1->CR2 = 16; // APB1 16 MHz
    I2C1->CCR = 80; 
    I2C1->TRISE = 17; 
    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1
}

// I2C write function
void I2C1_Write(uint8_t address, uint8_t data) 
	{
    while (I2C1->SR2 & I2C_SR2_BUSY); // Wait until bus is free
    I2C1->CR1 |= I2C_CR1_START; // START condition generation
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for START condition

    I2C1->DR = address; // Send slave address
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for address ACK
    (void)I2C1->SR2; // Clear ADDR flag

    I2C1->DR = data; // Send data
    while (!(I2C1->SR1 & I2C_SR1_BTF)); // Wait for data transmission

    I2C1->CR1 |= I2C_CR1_STOP; // STOP condition generation
}

// LCD initialization
void LCD_Init(void) 
	{
    delay(50); // Wait for LCD to power up
    LCD_Send_Command(0x33); // Initialize in 4-bit mode
    LCD_Send_Command(0x32); // Set 4-bit mode
    LCD_Send_Command(0x28); // 2-line, 5x8 dots
    LCD_Send_Command(0x0C); // Display ON, Cursor OFF
    LCD_Send_Command(0x06); // Increment cursor
    LCD_Send_Command(0x01); // Clear display
    delay(5); // Delay for clear display
}

// LCD send command function
void LCD_Send_Command(uint8_t command) 
	{
    uint8_t highNibble = command & 0xF0;
    uint8_t lowNibble = (command << 4) & 0xF0;

    I2C1_Write(LCD_ADDRESS, highNibble | 0x0C); 
    I2C1_Write(LCD_ADDRESS, highNibble | 0x08); 
    delay(2); 

    I2C1_Write(LCD_ADDRESS, lowNibble | 0x0C); 
    I2C1_Write(LCD_ADDRESS, lowNibble | 0x08); 
    delay(2); 
}

// LCD send string function
void LCD_Send_String(char *str) 
	{
    while (*str) {
        uint8_t data = *str;
        uint8_t highNibble = data & 0xF0;
        uint8_t lowNibble = (data << 4) & 0xF0;

        I2C1_Write(LCD_ADDRESS, highNibble | 0x0D);
        I2C1_Write(LCD_ADDRESS, highNibble | 0x09); 
        delay(1); 

        I2C1_Write(LCD_ADDRESS, lowNibble | 0x0D); 
        I2C1_Write(LCD_ADDRESS, lowNibble | 0x09); 
        delay(1); 

        str++;
    }
}
	
void delay(uint32_t ms) 
{
    for (volatile uint32_t i = 0; i < ms * 4000; i++); 
}