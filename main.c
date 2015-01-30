#include <stdint.h>
#include <string.h>

#include "main.h"
#include "ext_include/stm32f0xx.h"
#include "own_std.h"

void kakka_delay(uint32_t i)
{
	while(i--)
		__asm__ __volatile__ ("nop");
}

int v_mult = 5000;
int v_div = 65535;

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

// DMA priorities 0 (lowest) .. 3 (highest)
#define UART_TX_DMA_PRIORITY 1

#define TX_BUF_SIZE 100
char tx_buf[TX_BUF_SIZE+1];
#define RX_BUF_SIZE 50
char rx_buf[RX_BUF_SIZE+1];
int rx_point;


void usart_dma_tx(const char* buf, uint16_t n)
{
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR);
	DMA1_Channel2->CMAR = (uint32_t)buf;
	DMA1_Channel2->CNDTR = n;
	// Map USART1 TX to DMA channel 2, 8bit/8bit, memory increment, read from memory, enable
	DMA1_Channel2->CCR = (UART_TX_DMA_PRIORITY << 12) | (1<<7) | (1<<4);
	DMA1_Channel2->CCR |= DMA_CCR_EN;
}


void usart_print(const char *buf, int nmax)
{
	int n = o_strnlen(buf, nmax);
	usart_dma_tx(buf, n);
}


// SETV 4200
// SETI 20000
// SETI -20000
// DSCH
// CHA
// OFF
// MEAS

uint16_t v_setpoint = 3300;
int16_t i_setpoint = 0;


typedef struct
{
	uint16_t t_ntc;
	uint16_t v_sense;
	uint16_t v_direct;
	int16_t i_ref;
	int16_t i_meas;
	uint16_t t_cpu;
} __attribute__((packed)) adc_val_t;

volatile adc_val_t adc_data[16];

void blink()
{
	GPIOA->ODR = 0b00UL << 13;
	kakka_delay(400000);
	GPIOA->ODR = 0b01UL << 13;
	kakka_delay(400000);
	GPIOA->ODR = 0b10UL << 13;
	kakka_delay(400000);
	GPIOA->ODR = 0b11UL << 13;
	kakka_delay(400000);
}



void report_meas()
{
	int i;
	int v_sense, i_ref, i_meas;
	v_sense = 0;
	i_ref = 0;
	i_meas = 0;
	for(i = 0; i < 16; i++)
	{
		v_sense += adc_data[i].v_sense;
		i_ref += adc_data[i].i_ref;
		i_meas += adc_data[i].i_meas;
	}

	char* out = tx_buf;
	out = o_str_append(out, ";MEAS V=");
	out = o_utoa16(v_sense, out);
	out = o_str_append(out, " I=");
	out = o_utoa16(i_meas, out);
	out = o_str_append(out, " REF=");
	out = o_utoa16(i_ref, out);
	out = o_str_append(out, ";");
	*out = 0;
	usart_print(tx_buf, TX_BUF_SIZE);

//	int i_actual = i_meas - i_ref;
//	int v_actual = (v_sense*v_mult)/v_div;

}

#define OWN_ID 1

void handle_message()
{
	int id = -1;
	int val = 0;
	char* p_msg;
	char* p_val;
	if(rx_buf[0] != '@')
		return;
	p_msg = o_atoi_append(rx_buf, &id);
	if(id != OWN_ID)
	{
//		usart_print(";ID MISMATCH;", 20);
		return;
	}

	if(*p_msg++ != ':')
	{
//		usart_print(";COLON MISSING;", 20);
		return;
	}

	if((p_val = o_str_cmp(p_msg, "SETV")))
	{
		o_atoi_append(p_val, &val);
		if(val < 100 || val > 4500)
		{
			usart_print(";SETV OOR;", 20);
		}
		else
		{
			usart_print(";SETV OK;", 20);
			v_setpoint = val;
		}
	}
	else if((p_val = o_str_cmp(p_msg, "SETI")))
	{
		o_atoi_append(p_val, &val);
		if(val < -25000 || val > 25000)
		{
			usart_print(";SETI OOR;", 20);
		}
		else
		{
			usart_print(";SETI OK;", 20);
			i_setpoint = val;
		}
	}
	else if((p_val = o_str_cmp(p_msg, "MEAS")))
	{
		report_meas();
	}
	else if((p_val = o_str_cmp(p_msg, "DSCH")))
	{
		usart_print(";DSCH OK;", 20);
		cur_state = STATE_DSCH;
	}
	else if((p_val = o_str_cmp(p_msg, "CHA")))
	{
		usart_print(";CHA OK;", 20);
		cur_state = STATE_CHA;
	}
	else if((p_val = o_str_cmp(p_msg, "OFF")))
	{
		usart_print(";OFF OK;", 20);
		cur_state = STATE_OFF;
	}
}

void uart_rx_handler()
{
	char byte = USART1->RDR;
	if(USART1->ISR & USART_ISR_ORE)
	{
		USART1->ICR |= USART_ICR_ORECF;
		usart_print(";COMMERR;", 20);
	}

	if(byte == ';')
	{
		// rx_point holds the last position.
		rx_buf[rx_point] = 0;
//		usart_print(rx_buf, RX_BUF_SIZE);
		handle_message();
		rx_point = 0;
	}
	else if(byte >= 32 && byte < 128)
	{
		// Ignore control characters and 8-bit characters.
		if(byte >= 'a' && byte <= 'z')
			byte = byte - 'a' + 'A';
		rx_buf[rx_point] = byte;
		rx_point++;
		if(rx_point >= RX_BUF_SIZE-1)
			rx_point = 0;
	}

}


#define DO_OFF()   {GPIOA->ODR &= ~(0b1100UL);}
#define DO_BUCK()  {DO_OFF(); GPIOA->ODR |= 0b0100UL;}
#define DO_BOOST() {DO_OFF(); GPIOA->ODR |= 0b1000UL;}

void error()
{
	DO_OFF();
	__disable_irq();
	while(1)
	{
		usart_print(";ERROR;", 20);
		kakka_delay(40000000);
	}
}

void error2()
{
	DO_OFF();
	__disable_irq();
	while(1)
	{
		usart_print(";MUOVER;", 20);
		kakka_delay(40000000);
	}
}

int main()
{
	// Change to 48 MHz clock
	RCC->CFGR = 0b1010UL << 18; // PLL x12 (because of /2 prediv)
	RCC->CR |= 1UL << 24; // PLL on
	RCC->CFGR |= 0b10; // Change PLL to system clock

	RCC->AHBENR |= 1UL<<22 | 1UL<<18 | 1UL<<17 | 1UL; // Port F,B,A IO clock enable; DMA clock enable.
	RCC->APB1ENR |= 1UL<<1; // TIM3 clock enable.
	RCC->APB2ENR = (1UL << 14) | (1UL << 9); // USART1 and ADC


	// PORTA:
	// 13,14 GPO: LEDS
	GPIOA->ODR = 0;
	GPIOA->MODER = 0b0101UL << 26 /*LED*/ | 0b1111UL /*PA0,1 analog*/ |
		0b111111UL << 8 /*PA4,5,6 analog*/ | 0b0101UL << 4 /*PA2,3 buck/boost ena pins*/;
	GPIOA->MODER |= 0b10UL << 14;  // PA7 in Alternate Function mode: TIM3_CH2
	GPIOB->MODER = 0b10UL << 2;   // PB1 in Alternate Function mode: TIM3_CH4
	GPIOA->AFR[0] = 0b0001UL << 28; // PA7 Alternate Function selection: AF1 = TIM3_CH2
	GPIOB->AFR[0] = 0b0001UL << 4;  // PB1 Alternate Function selection: AF1 = TIM3_CH4

	// TIMER3 (PWM outputs)
	TIM3->CR1 = 1UL; // Counter enable
	TIM3->CCMR1 = 0b01100000UL << 8; // Channel2 in PWM mode 1
	TIM3->CCMR2 = 0b01100000UL << 8; // Channel4 in PWM mode 1
	TIM3->CCER = 1UL<<12 | 1UL<<4; // Channels2,4 enable
	// Prescaler 0 (0+1 = 1) by default

	TIM3->CCR2 = 10000;
	TIM3->CCR4 = 50000;


//	GPIOB->MODER = 0b1010UL << 16; // Timer 16&17 outputs (PORTB 8&9)
//	GPIOB->AFR[1] = 0b00100010; // AF2 for both PORTB 8 & 9 (timers)

	// USART1
	GPIOA->MODER |= 0b1010UL << 18; // TX and RX pins to Alternate Function mode.
//	GPIOA->OSPEEDR |= 0b11UL << 18;
	GPIOA->AFR[1] |= 0b0001UL << 4;
	GPIOA->AFR[1] |= 0b0001UL << 8;

// RX interrupt = RXNE. RXNEIE = enable control bit.

	// 48 MHz system clock, 16x oversampling (OVER8 = 0)
	USART1->BRR = 0x1388; // 0x1388 = 9600, 0x1a1 = 115200 bps
	USART1->CR1 = 0b101101; // RXNEIE + transmitter & receiver enable, usart enable, defaults (zeroes) good.
	USART1->CR3 = (1<<7); // DMA enable transmitter


	// Enable DMA: channel 1 for ADC.
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(adc_data);
	DMA1_Channel1->CNDTR = 6*16;
	DMA1_Channel1->CCR = 0b011010110101000UL; // very high prio, 16b->16b, MemIncrement, circular, transfer error interrupt, enable.

	// Enable and self-calibrate ADC.
	ADC1->CFGR2 = 0b10UL << 30; // PCLK/4, 12 MHz clock
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0);
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
//	ADC1->IER |= ADC_IER_AWDIE;
	ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;
	ADC1->SMPR = 0b101UL; // Sampling time = 55.5 ADC clock cycles
	ADC1->TR = (4050 << 16) | (0); // Watchdog thresholds

	ADC1->CHSELR = 0b00000000000000001UL; // 0,1,4,5,6,16(inttemp)
//	ADC1->CHSELR = 0b10000000001110011UL; // 0,1,4,5,6,16(inttemp)
	ADC->CCR = ADC_CCR_TSEN; // temperature sensor enable

	DMA1_Channel1->CCR |= 1UL;

	ADC1->CR |= ADC_CR_ADSTART;

	// TODO: Protection for DMA blockage

//	uint16_t v_set = 4200;
//	int32_t i_set = 5000;

	GPIOA->ODR = 0b11UL << 13;

	NVIC_EnableIRQ(USART1_IRQn);
	__enable_irq();
//	USART1->TDR = 'a';

	blink();
	usart_print("hei\n\r", 10);

	while(1)
	{
/*		GPIOA->ODR = 0b00UL << 13;
		kakka_delay(4000000);
		GPIOA->ODR = 0b01UL << 13;
		kakka_delay(4000000);
		GPIOA->ODR = 0b10UL << 13;
		kakka_delay(4000000);
		GPIOA->ODR = 0b11UL << 13;
		kakka_delay(4000000);
*/

//		TIM3->CCR4 = adc_data[0].t_ntc*16;
//		TIM3->CCR2 = adc_data[15].t_ntc*16;

//		USART1->TDR = 'a';
//		usart_print(teksti);
/*
		if(i_setpoint < 0 && cur_state == STATE_CHA)
		{
			error();
		}

		if(i_setpoint > 0 && cur_state == STATE_DSCH)
		{
			error();
		}

		if(v_setpoint > MAX_VOLTAGE || v_setpoint < MIN_VOLTAGE)
		{
			error();
		}
*/
	}
}

