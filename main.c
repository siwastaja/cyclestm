#include <stdint.h>
#include <string.h>

#include "main.h"
#include "ext_include/stm32f0xx.h"

void kakka_delay(uint32_t i)
{
	while(i--)
		;
}

// USART1_TX DMA request channel 2, USART1_RX DMA request channel 3
// Can be remapped to 4, 5 respectively by setting a bit in SYSCFG_CFGR1.

// DMA priorities 0 (lowest) .. 3 (highest)
#define UART_TX_DMA_PRIORITY 1

void usart_dma_tx(char* buf, uint16_t n)
{
	DMA1_Channel2->CPAR = (uint32_t)USART1->TDR;
	DMA1_Channel2->CMAR = (uint32_t)buf;
	DMA1_Channel2->CNDTR = n;
	// Map USART1 TX to DMA channel 2, 8bit/8bit, memory increment, read from memory, enable
	DMA1_Channel2->CCR = (UART_TX_DMA_PRIORITY << 12) | (1<<7) | (1<<4);
	DMA1_Channel2->CCR |= DMA_CCR_EN;

}

void usart_print(char *buf)
{
	int n = strlen(buf);
	if(n > 65535)
		return;
	usart_dma_tx(buf, n);
}

uint16_t v_mult = 5;
uint16_t v_div = 4;

int32_t i_mult = 20;
int32_t i_div = 1;

#define STATE_OFF  0
#define STATE_CHA  1
#define STATE_DSCH 2
#define STATE_ERR  3

uint8_t cur_state = STATE_OFF;

#define MODE_CC 0
#define MODE_CV 1

#define MIN_VOLTAGE 100
#define MAX_VOLTAGE 4800

int main()
{
	// Change to 48 MHz clock
	RCC->CFGR = 0b1010UL << 18; // PLL x12 (because of /2 prediv)
	RCC->CR |= 1UL << 24; // PLL on
	RCC->CFGR |= 0b10; // Change PLL to system clock

	RCC->AHBENR |= 1UL<<19 | 1UL<<18; // Port C,B IO clock enable
	GPIOC->MODER = 0b01UL << 28;
	GPIOB->MODER = 0b1010UL << 16; // Timer 16&17 outputs (PORTB 8&9)
	GPIOB->AFR[1] = 0b00100010; // AF2 for both PORTB 8 & 9 (timers)

	// Enable DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// USART1
	RCC->AHBENR |= 1UL << 17;
	RCC->APB2ENR |= 1UL << 14;
	GPIOA->MODER |= 0b10UL << 20; // yes or no??
	GPIOA->MODER |= 0b10UL << 18;
	GPIOA->OSPEEDR |= 0b11UL << 18;
	GPIOA->AFR[1] |= 0b0001UL << 4;
	GPIOA->AFR[1] |= 0b0001UL << 8;

	// 48 MHz system clock, 16x oversampling (OVER8 = 0)
	// 115200 bps, 0.08% error
	USART1->BRR = 0x1a1;
	USART1->CR1 = 0b1101; // transmitter & receiver enable, usart enable, defaults (zeroes) good.
	// TX DMA enable.
	USART1->CR3 = (1<<7);

	// Enable and self-calibrate ADC.
	RCC->APB2ENR |= 1UL << 9;
	ADC1->CFGR2 = 0b10UL << 30; // PCLK/4, 12 MHz clock
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0);
	ADC1->CR |= ADC_CR_ADCEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
	ADC1->IER |= ADC_IER_AWDIE;
	ADC1->CFGR1 = ADC_CFGR1_AWDEN | ADC_CFGR1_CONT | ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN; // All channels watchdoged.
	ADC1->SMPR = 0b101UL; // Sampling time = 55.5 ADC clock cycles
	ADC1->TR = (4050 << 16) | (0); // Watchdog thresholds
	ADC1->CHSELR = 0b111111111UL;
	// ADC1->CCR would enable VBat, int temp sensor & int vref

	ADC1->CR |= ADC_CR_ADSTART;

	// TIM16CH1 (PB8) = I+SET 1
	// TIM17CH1 (PB9) = I-SET 1
	// TIM2CH3 (PB10) = I+SET 2
	// TIM2CH4 (PB11) = I-SET 2
	// TIM1CH1N (PB13) = I+SET 3
	// TIM1CH2N (PB14) = I-SET 3

	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM16->CR1 = TIM_CR1_ARPE;
	TIM16->CCMR1 = 0b110UL << 4 | OC1PE; // PWM mode 1
	TIM16->CCER = 1; // output enable
	TIM16->PSC = 0; // prescaler = 1.
	TIM16->ARR = 4096; // 11.7 kHz
	TIM16->CCR1 = 1234;
	TIM16->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_OSSR;

	TIM16->EGR |= 1; // Initialize timer shadow registers or something like that.
	TIM16->CR1 |= 1; // Enable counter.


	struct
	{
		int16_t i_meas;
		int16_t i_ref;
		uint16_t v_meas;
		uint16_t t_ext;
		uint16_t t_int;
	} adc_val;

	uint16_t v_set = 4200;
	int32_t i_set = 5000;

	while(1)
	{
		if(i_set < 0 && cur_state == STATE_CHA)
		{
			error();
		}

		if(i_set > 0 && cur_state == STATE_DSCH)
		{
			error();
		}

		if(v_set > MAX_VOLTAGE || v_set < MIN_VOLTAGE)
		{
			error();
		}

		int32_t i_set_adj = i_set;

		GPIOC->ODR = 1UL << 14;
		kakka_delay(800000);
		GPIOC->ODR = 0UL;
		kakka_delay(800000);
		char buffet[200];
		int32_t i_act = ((int32_t)i_meas - (int32_t)i_ref) * i_mult / i_div;
		uint16_t mv_act = (uint32_t) v_meas * v_mult / v_div;
		sprintf(buffet, "Iref=%u Imeas=%u Vmeas=%u Text=%u Tint=%u Iact=%d mVact=%u\r\n", adc_vals.i_ref, adc_vals.i_meas,
		adc_vals.v_meas, adc_vals.t_ext, adc_vals.t_int, i_act, mv_act);
		usart_print(buffet);
	}
}

