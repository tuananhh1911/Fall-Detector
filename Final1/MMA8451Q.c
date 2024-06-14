#include "stm32f10x.h"
#include "MMA8451Q.h"

void GPIO_Init()
{
	// Enable GPIOB clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

  // Configure PB6 and PB7 for I2C1 SCL and SDA
  GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
  GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7;
  GPIOB->ODR |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7;
}

void I2C1_Init()
{
	// Enable I2C1 clock
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  // Reset I2C1
  RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

  // Configure I2C1
  I2C1->CR2 |= 36;
  I2C1->CCR |= I2C_CCR_FS | I2C_CCR_DUTY | (30 & I2C_CCR_CCR);
	I2C1->OAR1 = 180;
  I2C1->TRISE = 37;
  I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_Write(uint8_t addr, uint8_t reg, uint8_t data)
{
    // Send start condition
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // Send address
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    // Send register
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // Send data
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // Send stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_Read(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length)
{
    // Send start condition
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // Send address for write
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    // Send register
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // Send repeated start condition
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // Send address for read
    I2C1->DR = addr | 0x01;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    while (length)
    {
        if (length == 1)
        {
            // Disable ACK and generate stop condition
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }

        // Wait for data to be received
        while (!(I2C1->SR1 & I2C_SR1_RXNE));

        // Read data
        *buffer++ = I2C1->DR;
        length--;
    }

    // Enable ACK
    I2C1->CR1 |= I2C_CR1_ACK;
}