#include "stm32f10x.h"
#include "MMA8451Q.h"
#include "LCD.h"
#include <stdio.h>
#include "math.h"
#include "stdbool.h"

void SystemClock_Config(void);
void SysTick_Init(void);
void LED_Init(void);
void toggle_LED1(void);
void toggle_LED2(void);
void Switch_Init(void);
void EXTI_Config(void);

#define MMA8451Q_ADDR (0x1D << 1)
#define LED1_Pin 0
#define LED2_Pin 1
#define LED_GPIO_PORT GPIOB

volatile float ax, ay, az;
volatile int16_t x, y, z;
uint8_t buffer[6];
volatile bool isFall = false;
char buf[16];

//Trang thai cua he thong
volatile uint8_t Set = 0;//khong chay
volatile uint8_t Reset = 0;//khong reset
volatile uint32_t ticks = 0;

int main(void)
{
	SystemClock_Config();
	SysTick_Init();
	GPIO_Init();
	I2C1_Init();
	lcd_init();
	LED_Init();
	Switch_Init();
	EXTI_Config();
	
	I2C1_Write(MMA8451Q_ADDR, 0x2A, 0x00); //Put device in standby mode
	I2C1_Write(MMA8451Q_ADDR, 0x0E, 0x00); //Set range to 2g
	I2C1_Write(MMA8451Q_ADDR, 0x2A, 0x01); //Put device in active mode
	
	while(1)
	{
		if(Set == 1)//When SW1 is pressed
		{	
			//Toggle LED1
			toggle_LED1();	
			//Read accelerometer data
			I2C1_Read(MMA8451Q_ADDR, 0x01, buffer, 6);
		
			x = (int16_t)(buffer[0] << 8 | buffer[1]);
			y = (int16_t)(buffer[2] << 8 | buffer[3]);
			z = (int16_t)(buffer[4] << 8 | buffer[5]);
		
			ax = (float)x / 4096.0f * 9.81f;
			ay = (float)y / 4096.0f * 9.81f;
			az = (float)z / 4096.0f * 9.81f;
		
			//Fall detected 
			if(sqrt(ax*ax + ay*ay + az*az) > 5.0)
			{
				isFall = true;
			} else {
				isFall = false;
			}
			//Display isFall on LCD
			lcd_clear();
			lcd_set_cursor(0,0);
			sprintf(buf, "isFall: %d", isFall);
			lcd_print(buf);
		
			for(volatile int i = 0; i < 100000; i++); //Delay
		}
		if (Reset == 1) //When SW2 is pressed
		{
			Set = 1;
			Reset = 0;
			isFall = false;
		}		
	}
}

void SystemClock_Config()
{
	// Kich hoat HSE (High-Speed External) oscillator
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY)){} // Doi cho HSE san sang, HSERDY Flag set len 1

  // Cau hinh va kich hoat PLL
  RCC->CFGR |= RCC_CFGR_PLLSRC; // Chon HSE lam nguon PLL
  RCC->CFGR |= RCC_CFGR_PLLMULL9; // PLL x 9 de dat 72 MHz

  RCC->CR |= RCC_CR_PLLON; // Kich hoat PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)); // Doi cho PLL san sang

  // Cau hinh cac bo chia AHB, APB1 và APB2
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB = SYSCLK khong chia
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 = HCLK chia 2
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = HCLK không chia

  // Chuyen he thong Clock nguon sang PLL
  FLASH->ACR |= FLASH_ACR_LATENCY_2; // Cau hinh Flash latency 2 wait states
  RCC->CFGR |= RCC_CFGR_SW_PLL; // Chon PLL lam SYSCLK
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Doi cho PLL duoc su dung lam SYSCLK
}

void SysTick_Init() {
    SysTick->CTRL = 0;
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void LED_Init() {
  // Bat clock cho GPIOB
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
  // Cau hình PB0 va PB1 là output push-pull
  GPIOB->CRL &= ~((GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | (GPIO_CRL_CNF1 | GPIO_CRL_MODE1));
  GPIOB->CRL |= (GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0);
}

void toggle_LED1(){
	GPIOB->ODR ^= LED1_Pin;
	for(int i = 0; i < 500000; i++); //Tan so 1Hz
}

void toggle_LED2() {
	GPIOB->ODR ^= LED2_Pin;
	for(int i = 0; i < 250000; i++); //Tan so 2Hz
}	

void Switch_Init() {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// Cau hinh PA0 PA1 la input pull up
  GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0 | GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
  GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0;
  GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1; // Kích ho?t pull-up
}

void EXTI_Config() {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Bat clock cho AFIO

    // Ket noi chân EXTI0 voi chân PA0 và EXTI1 voi chân PA1
    AFIO->EXTICR[0] &= ~(AFIO_EXTICR1_EXTI0 | AFIO_EXTICR1_EXTI1);
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA | AFIO_EXTICR1_EXTI1_PA;

    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1; // Kích ho?t interrupt cho EXTI0 và EXTI1
    EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1; // Cài d?t ngã xu?ng làm trigger cho EXTI0 và EXTI1

    NVIC_EnableIRQ(EXTI0_IRQn); // Kích ho?t NVIC cho EXTI0
    NVIC_SetPriority(EXTI0_IRQn, 0); // Ð?t d? uu tiên cho EXTI0
    NVIC_EnableIRQ(EXTI1_IRQn); // Kích ho?t NVIC cho EXTI1
    NVIC_SetPriority(EXTI1_IRQn, 0); // Ð?t d? uu tiên cho EXTI1
}

void SysTick_Handler(void) {
    static uint32_t ticksB0 = 0;
    static uint32_t ticksB1 = 0;
	
    ticks++;

    if ((++ticksB0 >= 1000) & (Set == 1)) 
		{ 
			ticksB0 = 0;
      GPIOB->ODR ^= GPIO_ODR_ODR0; 
			
    } else if (Set ==0) {
			GPIOB->ODR &= ~(1<<0);
		}
  	if ((++ticksB1 >= 500) & (isFall)) 
		{ 
			ticksB1 = 0;
			GPIOB->ODR ^= GPIO_ODR_ODR1; 
		}
		else if (!isFall) {
			GPIOB->ODR &= ~(1<<1);
		}
}

void EXTI1_IRQHandler(void)//reset
{
	EXTI->PR |= EXTI_PR_PR1;
	Reset = 1;
}

void EXTI0_IRQHandler(void) //switch set  
{
    EXTI->PR |= EXTI_PR_PR0;
		Set = !Set;
}		
