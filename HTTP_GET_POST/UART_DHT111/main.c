#include "stm32f10x.h" 
#include "systick_delay.h"
#include "gpio.h"


void DHT_Run(void);
uint8_t DHT_Read(void);
uint8_t DHT_CheckResponse(void);
void DMA_Config(void);
void UART_Config(void);

uint8_t Data[4]; //Data[0]: HUM1, Data[1]: HUM2, Data[2]: TEM1, Data[3]: TEM2;


int main(void)
{
	 UART_Config();
	 DMA_Config();
	while(1)
	{
		DHT_Run();
		delayMs(1000);
		DMA1_Channel7->CCR |= 1; //enable DMA channel 7
		while(!((DMA1->ISR >> 25) & 1)); //Channel 7 transfer complete
		DMA1->IFCR |= 1<<25; //Channel 7 transfer complete clear 
		DMA1_Channel7->CCR &= ~1; //disable DMA channel 7
		delayMs(5000);
	}
}

uint8_t DHT_Read(void)
{
	uint8_t var;
	for(int i = 0; i < 8; i++)
	{
		while(!GPIO_ReadPin(GPIOB, 9)); //Wait for PB9 high (end of 50ms signal)
		delayUs(40);
		if(GPIO_ReadPin(GPIOB, 9))
		{
			var = (var << 1)|(0x01); //Write 1 to var
			while(GPIO_ReadPin(GPIOB, 9)); //Wait for PB9 low (end of 70ms signal)
		}
		else
		{
			var = (var << 1); //Write 0 to var
		}
	}
	return var;
}

uint8_t DHT_CheckResponse(void)
{
	while(GPIO_ReadPin(GPIOB, 9)); //Wait till the end of 10ms high signal
	delayUs(90);
	if(!GPIO_ReadPin(GPIOB, 9))
	{
		return 0; 
	}
	else
	{
		while(GPIO_ReadPin(GPIOB, 9));//Wait till the end 80ms high signal
		return 1;
	}
}

void DHT_Run(void)
{
	uint8_t HUM1 = 0, HUM2 = 0, TEM1 = 0, TEM2 = 0, SUM = 0;
	//Config GPIO
	GPIO_Init(GPIOB, 9, OUT_HIGH, GP_OD);
	
	//Transmit start signal
	GPIO_WritePin(GPIOB, 9, 0);
	delayMs(20);
	GPIO_WritePin(GPIOB, 9, 1);
	
	if(DHT_CheckResponse())
	{
		HUM1 = DHT_Read();
		HUM2 = DHT_Read();
	
		TEM1 = DHT_Read();
		TEM2 = DHT_Read();

		SUM = DHT_Read();
	}
	if(SUM == HUM1 + HUM2 + TEM1 + TEM2)
	{
		Data[0] = HUM1;
		Data[1] = HUM2;
		Data[2] = TEM1;
		Data[3] = TEM2;
	}
}


void UART_Config(void)
{
	//Config USART2
	RCC->APB1ENR |= 1<<17; //Clock USART2 enable
	RCC->APB2ENR |= 1<<2; //Clock PORTA enable
	GPIOA->CRL &= ~(1<<8 | 1<<9 | 1<<10 | 1<<11); //Config PA2 as TX
	GPIOA->CRL |= (1<<8 | 1<<9 | 1<<11);
	GPIOA->CRL &= ~(1<<12 | 1<<13 | 1<<14 | 1<<15); //Config PA3 as RX
	GPIOA->CRL |= (1<<15);
	USART2->BRR = 0xEA6;	//Set baud rate 9600
	USART2->CR1 |= 1<<13 | 1<<3; //Enable USART2 & transmitter
}

void DMA_Config(void)
{
	//Config DMA
	RCC->AHBENR |= 1<<0; //enable clock DMA1 
	USART2->CR3 |= 1<<7; //enable DMA transmit
	DMA1_Channel7->CNDTR = 4; //set counter
	//enable Memory increment mode & Circular mode & Data transfer direction & Transfer complete interrupt 
	DMA1_Channel7->CCR |= (1<<7 | 1<<5 | 1<<4 | 1<<1);
	DMA1_Channel7->CMAR = (uint32_t) Data; //Address of memory
	DMA1_Channel7->CPAR = (uint32_t)& USART2->DR; //Address of peripheral
}
