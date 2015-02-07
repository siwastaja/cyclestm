#define OWN_ID 1
#define OWN_ID_STR ";1:"

//#define DISABLE_TRIM

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
#define V_SHIFT 16

#define NUM_ADC_SAMPLES 64
#define ADC_SHIFT 2 // from 64 summed samples to 16-bit value
#define SECOND_FILT_LEN 128
#define SECOND_FILT_SHIFT 7


#define V_DIRECT_MAX 5500
#define V_DIRECT_MAX_DELTA 1000
int v_direct_mult = 6600;
#define V_DIRECT_SHIFT 16;

#define STATE_OFF  0
#define STATE_CHA  1
#define STATE_DSCH 2
#define STATE_ERR  3

volatile int cur_state = STATE_OFF;

volatile int mode_cv = 0;

#define MIN_VOLTAGE 100
#define MAX_VOLTAGE 4800

// DMA priorities 0 (lowest) .. 3 (highest)
#define UART_TX_DMA_PRIORITY 1

#define TX_BUF_SIZE 100
char tx_buf[TX_BUF_SIZE+1];
#define RX_BUF_SIZE 50
char rx_buf[RX_BUF_SIZE+1];
volatile int rx_point;

#define RED_ON()  {GPIOA->ODR &= ~(1UL << 14);}
#define RED_OFF() {GPIOA->ODR |= 1UL << 14;}
#define GREEN_ON()  {GPIOA->ODR &= ~(1UL << 13);}
#define GREEN_OFF() {GPIOA->ODR |= 1UL << 13;}


volatile int print_disallow;

void usart_print(const char *buf, int nmax)
{
	int timeout = 1000000;
	int n = o_strnlen(buf, nmax);

	if(n<1) return;

//	RED_ON();
	while(print_disallow)
	{
		timeout--;
		if(timeout == 0)
		{
			DMA1->IFCR |= 0b100000;
			break;
		}
	}
//	RED_OFF();
//	if(DMA1_Channel2->CCR & DMA_CCR_EN)
//	{
//		while(!((DMA1->ISR)&0b100000)) ;
//	}
//	DMA1->IFCR |= 0b100000;

	__disable_irq();
	GPIOA->MODER |= 0b10UL << 18; // TX pin to Alternate Function mode.
	print_disallow = 1;
	kakka_delay(50);
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR);
	DMA1_Channel2->CMAR = (uint32_t)buf;
	DMA1_Channel2->CNDTR = n;
	// Map USART1 TX to DMA channel 2, 8bit/8bit, memory increment, read from memory, transfer complete interrupt enable
	DMA1_Channel2->CCR = (UART_TX_DMA_PRIORITY << 12) | (1<<7) | (1<<4) | (1<<3) | (1<<1);
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1->IFCR |= 0b100000;
	__enable_irq();
}




// SETV 4200
// SETI 20000
// SETI -20000
// DSCH
// CHA
// OFF
// MEAS

int v_setpoint = 3300;
int i_setpoint;
int i_override;
int i_trim; // = 0
int i_midpoint_trim; // = 0
int i_stop;
int v_stop;

#define MAX_I_TRIM 2000
#define MIN_I_TRIM -2000
#define COARSE_MIDPOINT_TRIM -1000
#define MIN_MIDPOINT_TRIM -1000
#define MAX_MIDPOINT_TRIM 1000

int v_sense_midflt[SECOND_FILT_LEN];
int i_meas_midflt[SECOND_FILT_LEN];
int t_ntc_midflt[SECOND_FILT_LEN];

int v_sense;
int i_meas;
int t_ntc;
int v_direct;


int i_trim_ignore_cnt;
#define I_TRIM_IGNORE_CNT_VAL 20

typedef struct
{
	uint16_t t_ntc;
	uint16_t v_sense;
	uint16_t v_direct;
	int16_t i_ref;
	int16_t i_meas;
	uint16_t t_cpu;
} __attribute__((packed)) adc_val_t;

volatile adc_val_t adc_data[NUM_ADC_SAMPLES];

#define DO_OFF()   {GPIOA->ODR &= ~(0b1100UL);}
#define DO_BUCK()  {DO_OFF(); GPIOA->ODR |= 0b0100UL;}
#define DO_BOOST() {DO_OFF(); GPIOA->ODR |= 0b1000UL;}

#define SET_POSLIMIT(n) {TIM3->CCR4 = (n);}
#define SET_NEGLIMIT(n) {TIM3->CCR2 = (n);}

#define I_MIDPOINT (32768+COARSE_MIDPOINT_TRIM)
#define MAX_POS_I 29000
#define MIN_NEG_I -29000
#define IDLE_POS_I (I_MIDPOINT+i_midpoint_trim+2000)
#define IDLE_NEG_I (I_MIDPOINT+i_midpoint_trim-2000)


void blink()
{
	RED_OFF(); GREEN_OFF();
	kakka_delay(400000);
	RED_ON();
	kakka_delay(400000);
	RED_OFF(); GREEN_ON();
	kakka_delay(400000);
	RED_OFF(); GREEN_OFF();
	kakka_delay(400000);
}



void error(char code)
{
	static char msg[12] = ";FATAL x;\0\0";
	DO_OFF();
	__disable_irq();
	SET_POSLIMIT(I_MIDPOINT+1000);
	SET_NEGLIMIT(I_MIDPOINT-1000);

	msg[7] = code;

	while(1)
	{
		blink();
		print_disallow = 0;
		usart_print(msg, 12);
		kakka_delay(1000000);
	}
}



void set_hw_curr_limits()
{
	if(cur_state == STATE_CHA)
	{
		if(i_override < 0 || i_override > MAX_POS_I)
			error('2');

		int val = I_MIDPOINT + i_midpoint_trim + i_override;
#ifndef DISABLE_TRIM
		val += i_trim;
#endif

		if(i_midpoint_trim < MIN_MIDPOINT_TRIM || i_midpoint_trim > MAX_MIDPOINT_TRIM ||
		   val < I_MIDPOINT+MIN_MIDPOINT_TRIM || val > I_MIDPOINT+MAX_POS_I)
			error('3');

		SET_POSLIMIT(val);
		SET_NEGLIMIT(IDLE_NEG_I);
	}
	else if(cur_state == STATE_DSCH)
	{
		if(i_override > 0 || i_override < MIN_NEG_I)
			error('4');

		int val = I_MIDPOINT + i_midpoint_trim + i_override;

#ifndef DISABLE_TRIM
		val += i_trim;
#endif

		if(i_midpoint_trim < MIN_MIDPOINT_TRIM || i_midpoint_trim > MAX_MIDPOINT_TRIM ||
		   val < I_MIDPOINT+MIN_NEG_I || val > I_MIDPOINT+MAX_MIDPOINT_TRIM)
			error('5');

		SET_NEGLIMIT(val);
		SET_POSLIMIT(IDLE_POS_I);
	}
}

void off()
{
	cur_state = STATE_OFF;
	DO_OFF();
	SET_POSLIMIT(IDLE_POS_I);
	SET_NEGLIMIT(IDLE_NEG_I);
	i_trim = 0;
	i_trim_ignore_cnt = I_TRIM_IGNORE_CNT_VAL;

	blink();
	RED_OFF(); GREEN_OFF();
}

void set_current(int current)
{
	if((cur_state == STATE_DSCH && current > 0) ||
	   (cur_state == STATE_CHA && current < 0 ))
	{
		usart_print(";SETI SIGN ERR;\r\n", 30);
		return;
	}

	i_setpoint = current;

	if((cur_state == STATE_DSCH && i_override < current) ||
	   (cur_state == STATE_CHA && i_override > current))
		i_override = current;

	usart_print(";SETI OK;\r\n", 20);
	i_trim_ignore_cnt = I_TRIM_IGNORE_CNT_VAL;

}

void discharge()
{
	off();

	if(i_setpoint >= 0 || i_setpoint < MIN_NEG_I)
	{
		usart_print(";DSCH PARAM ERR;\r\n", 30);
		return;
	}
	usart_print(";DSCH OK;\r\n", 30);

	i_override = i_setpoint;

	DO_BOOST();
	cur_state = STATE_DSCH;
	RED_ON(); GREEN_OFF();

}

void charge()
{
	off();

	if(i_setpoint <= 0 || i_setpoint > MAX_POS_I)
	{
		usart_print(";CHA PARAM ERR;\r\n", 30);
		return;
	}
	usart_print(";CHA OK;\r\n", 30);

	i_override = i_setpoint;


	DO_BUCK();
	cur_state = STATE_CHA;
	GREEN_ON(); RED_OFF();
}

void calc_second_filt()
{
	int i;
	int calc_v_sense = 0;
	int calc_i_meas = 0;
	int calc_t_ntc = 0;
	for(i = 0; i < SECOND_FILT_LEN; i++)
	{
		calc_v_sense += v_sense_midflt[i];
		calc_i_meas += i_meas_midflt[i];
		calc_t_ntc += t_ntc_midflt[i];
	}

	calc_v_sense >>= SECOND_FILT_SHIFT;
	calc_i_meas >>= SECOND_FILT_SHIFT;
	calc_t_ntc >>= SECOND_FILT_SHIFT;

	v_sense = calc_v_sense;
	i_meas = calc_i_meas;
	t_ntc = calc_t_ntc;
}

void calc_adc_vals(int idx)
{
	int calc_v_sense = 0;
	int calc_i_ref = 0;
	int calc_i_meas = 0;
	int calc_ntc = 0;
	int calc_v_direct = 0;
	int i;

	for(i = 0; i < NUM_ADC_SAMPLES; i++)
	{
		calc_v_sense += adc_data[i].v_sense;
		calc_i_ref += adc_data[i].i_ref;
		calc_i_meas += adc_data[i].i_meas;
		calc_ntc += adc_data[i].t_ntc;
		calc_v_direct += adc_data[i].v_direct;
	}
	calc_v_sense >>= ADC_SHIFT; // don't combine with later shift, there is too little headroom in int.
	calc_i_meas >>= ADC_SHIFT;
	calc_i_ref >>= ADC_SHIFT;
	calc_ntc >>= ADC_SHIFT;
	calc_v_direct >>= ADC_SHIFT;
	calc_i_meas -= calc_i_ref;
	calc_v_sense *= v_mult;
	calc_v_direct *= v_direct_mult;
	calc_v_direct >>= V_DIRECT_SHIFT;
	calc_v_sense >>= V_SHIFT;

	v_sense_midflt[idx] = calc_v_sense;
	i_meas_midflt[idx] = calc_i_meas;
	t_ntc_midflt[idx] = calc_ntc;
	v_direct = calc_v_direct;
}

#define CV_DEADBAND 2
#define CV_REACT_RATE 1
#define I_TRIM_REACT_SLOWDOWN 3

void adjust_i_trim()
{
	if(i_trim_ignore_cnt)
	{
		i_trim_ignore_cnt--;
		return;
	}

	if(i_meas > i_override)
	{
		i_trim -= (i_meas-i_override)>>I_TRIM_REACT_SLOWDOWN;
		if(i_trim < MIN_I_TRIM)
			i_trim = MIN_I_TRIM;
	}
	else if(i_meas < i_override)
	{
		i_trim += (i_override-i_meas)>>I_TRIM_REACT_SLOWDOWN;
		if(i_trim > MAX_I_TRIM)
			i_trim = MAX_I_TRIM;
	}
}

void adjust_charge()
{
	if(v_sense > v_setpoint+CV_DEADBAND)
	{
		i_override -= (v_sense-v_setpoint)*CV_REACT_RATE;
		if(i_override < 0)
		{
			off();
		}
	}
	else if(v_sense < v_setpoint-CV_DEADBAND && i_override < i_setpoint)
	{
		i_override += (v_setpoint-v_sense)*CV_REACT_RATE;
		if(i_override >= i_setpoint)
			i_override = i_setpoint;
	}

	if(i_override > i_setpoint || i_override < 0)
		error('6');

	adjust_i_trim();

	mode_cv = (i_override != i_setpoint);

	if(i_override <= i_stop || v_sense >= v_stop)
	{
		off();
	}
}

void adjust_discharge()
{
	if(v_sense < v_setpoint-CV_DEADBAND)
	{
		i_override += (v_setpoint-v_sense)*CV_REACT_RATE;
		if(i_override > 0)
		{
			off();
		}
	}
	else if(v_sense > v_setpoint+CV_DEADBAND && i_override > i_setpoint)
	{
		i_override -= (v_sense-v_setpoint)*CV_REACT_RATE;
		if(i_override <= i_setpoint)
			i_override = i_setpoint;
	}

	if(i_override < i_setpoint || i_override > 0)
		error('7');

	adjust_i_trim();

	mode_cv = (i_override != i_setpoint);

	if(i_override >= i_stop || v_sense <= v_stop)
	{
		off();
	}
}

void report_params()
{
	char* out = tx_buf;
	out = o_str_append(out, OWN_ID_STR);

	out = o_str_append(out, " Vset=");
	out = o_utoa16(v_setpoint, out);
	out = o_str_append(out, " Iset=");
	out = o_itoa16(i_setpoint, out);
	out = o_str_append(out, " Istop=");
	out = o_itoa16(i_stop, out);
	out = o_str_append(out, " Vstop=");
	out = o_utoa16(v_stop, out);
	out = o_str_append(out, " Imidtrim=");
	out = o_itoa16(i_midpoint_trim, out);

	out = o_str_append(out, ";\r\n");
	*out = 0;
	usart_print(tx_buf, TX_BUF_SIZE);

}

void report_meas(int verbose)
{
	char* out = tx_buf;
	out = o_str_append(out, OWN_ID_STR);
	out = o_str_append(out, "MEAS ");
	if(cur_state == STATE_CHA)
		out = o_str_append(out, "CHA ");
	else if(cur_state == STATE_DSCH)
		out = o_str_append(out, "DSCH ");
	else if(cur_state == STATE_OFF)
		out = o_str_append(out, "OFF ");
	else
		error('8');

	out = o_str_append(out, mode_cv?"CV":"CC");

	out = o_str_append(out, " V=");
	out = o_utoa16(v_sense, out);
	out = o_str_append(out, " I=");
	out = o_itoa16(i_meas, out);
	out = o_str_append(out, " T=");
	out = o_utoa16(t_ntc, out);

	if(verbose)
	{
		out = o_str_append(out, " Vdirect=");
		out = o_utoa16(v_direct, out);
		out = o_str_append(out, " Tcpu=");
		out = o_utoa16(adc_data[NUM_ADC_SAMPLES-1].t_cpu, out);
		out = o_str_append(out, " Iset=");
		out = o_itoa16(i_override, out);
		out = o_str_append(out, " Itrim=");
		out = o_itoa16(i_trim, out);
	}

	out = o_str_append(out, ";\r\n");
	*out = 0;
	usart_print(tx_buf, TX_BUF_SIZE);

//	int i_actual = i_meas - i_ref;
//	int v_actual = (v_sense*v_mult)/v_div;

}

void handle_message()
{
	int id = -1;
	int val = 0;
	char* p_msg;
	char* p_val;


	if(rx_buf[0] != '@')
	{
//		usart_print(";@ MISMATCH;", 20);
		return;
	}
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

	if((p_val = o_str_cmp(p_msg, "SETV ")))
	{
		o_atoi_append(p_val, &val);
		if(val < 100 || val > 4500)
		{
			usart_print(";SETV OOR;\r\n", 20);
		}
		else
		{
			usart_print(";SETV OK;\r\n", 20);
			v_setpoint = val;
		}
	}
	else if((p_val = o_str_cmp(p_msg, "SETI ")))
	{
		o_atoi_append(p_val, &val);
		if(val < -25000 || val > 25000)
		{
			usart_print(";SETI OOR;\r\n", 20);
		}
		else
		{
			set_current(val);
		}
	}
	else if((p_val = o_str_cmp(p_msg, "SETVSTOP ")))
	{
		o_atoi_append(p_val, &val);
		if(val < 100 || val > 5000)
		{
			usart_print(";SETVSTOP OOR;\r\n", 20);
		}
		else
		{
			usart_print(";SETVSTOP OK;\r\n", 20);
			v_stop = val;
		}
	}
	else if((p_val = o_str_cmp(p_msg, "SETISTOP ")))
	{
		o_atoi_append(p_val, &val);
		if(val < -25000 || val > 25000)
		{
			usart_print(";SETISTOP OOR;\r\n", 20);
		}
		else
		{
			usart_print(";SETISTOP OK;\r\n", 20);
			i_stop = val;
		}
	}
	else if((p_val = o_str_cmp(p_msg, "SETTRIM ")))
	{
		o_atoi_append(p_val, &val);
		if(val < MIN_MIDPOINT_TRIM || val > MAX_MIDPOINT_TRIM)
		{
			usart_print(";SETTRIM OOR;\r\n", 30);
		}
		else
		{
			usart_print(";SETTRIM OK;\r\n", 30);
			i_midpoint_trim = val;
		}
	}
	else if((p_val = o_str_cmp(p_msg, "MEAS")))
	{
		report_meas(0);
	}
	else if((p_val = o_str_cmp(p_msg, "VERB")))
	{
		report_meas(1);
	}
	else if((p_val = o_str_cmp(p_msg, "PAR")))
	{
		report_params();
	}
	else if((p_val = o_str_cmp(p_msg, "DSCH")))
	{
		discharge();
	}
	else if((p_val = o_str_cmp(p_msg, "CHA")))
	{
		charge();
	}
	else if((p_val = o_str_cmp(p_msg, "OFF")))
	{
		usart_print(";OFF OK;\r\n", 20);
		off();
	}
}

void uart_dma_finished_handler()
{
	DMA1->IFCR |= 0b100000;
	kakka_delay(17000);
	GPIOA->MODER &= ~(0b11UL << 18); // TX pin to input mode
	print_disallow = 0;
}

void input_overvoltage_handler()
{
	error('N');
}

volatile int do_handle_message;

void uart_rx_handler()
{
//	static int checksum = 0;
//	static char prev_byte;

	char byte = USART1->RDR;
//	checksum += byte;
	if(USART1->ISR & USART_ISR_ORE)
	{
		USART1->ICR |= USART_ICR_ORECF;
//		usart_print(";COMMERR;", 20);
	}
	if(do_handle_message)
	{
		// Ignore incoming data if we are processing a command.
//		usart_print(";BUSY;", 20);
		return;
	}

	if(byte == ';')
	{
		// rx_point holds the last position.
		rx_buf[rx_point] = 0;
//		usart_print(rx_buf, RX_BUF_SIZE);
		do_handle_message = 1;
		rx_point = 0;
	}
	else if(byte >= 32 && byte < 128)
	{
		// Ignore control characters and 8-bit characters.
		// Buffer gets full? Just wrap it over, we will
		// lose data. So always begin commands with the flushing
		// ; character to be sure.
		if(byte >= 'a' && byte <= 'z')
			byte = byte - 'a' + 'A';
		rx_buf[rx_point] = byte;
		rx_point++;
		if(rx_point >= RX_BUF_SIZE-1)
			rx_point = 0;
	}

//	prev_byte = byte;
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

	off();

//	GPIOB->MODER = 0b1010UL << 16; // Timer 16&17 outputs (PORTB 8&9)
//	GPIOB->AFR[1] = 0b00100010; // AF2 for both PORTB 8 & 9 (timers)

	// USART1
	GPIOA->MODER |= 0b10UL << 20; // RX pin to Alternate Function mode.
//	GPIOA->PUPDR |= 0b10UL << 18; // TX pin pulldown

	GPIOA->AFR[1] |= 0b0001UL << 4;
	GPIOA->AFR[1] |= 0b0001UL << 8;

// RX interrupt = RXNE. RXNEIE = enable control bit.

	// 48 MHz system clock, 16x oversampling (OVER8 = 0)
//	USART1->BRR = 0x1388; // 9600 bps
	USART1->BRR = 0x1a1;  // 115200 bps
	USART1->CR1 = 0b101101; // RXNEIE + transmitter & receiver enable, usart enable, defaults (zeroes) good.
	USART1->CR3 = (1<<7); // DMA enable transmitter


	// Enable DMA: channel 1 for ADC.
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(adc_data);
	DMA1_Channel1->CNDTR = 6*NUM_ADC_SAMPLES;
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

//	ADC1->CHSELR = 0b00000000000000001UL; // 0,1,4,5,6,16(inttemp)
	ADC1->CHSELR = 0b10000000001110011UL; // 0,1,4,5,6,16(inttemp)
	ADC->CCR = ADC_CCR_TSEN; // temperature sensor enable

	DMA1_Channel1->CCR |= 1UL;

	ADC1->CR |= ADC_CR_ADSTART;

	// TODO: Protection for DMA blockage

	RED_OFF(); GREEN_OFF();

	int jes = 0;
	for(jes = 0; jes < 5; jes++)
		blink();


	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	__enable_irq();

/*	usart_print(".hei\n\r.", 10);
	usart_print("heimoijoojuukatohehheh siwa siwa johannes kakka\n\r.", 200);
	usart_print("hei\n\r.", 10);
	usart_print("hei\n\r", 10);
*/
//	int i = 0;
	while(1)
	{
		int sampleset;

		// Let the DMA handle NUM_ADC_SAMPLES from every channel.
		// Average those when they are available.
		for(sampleset = 0; sampleset < SECOND_FILT_LEN; sampleset++)
		{
			while(!((DMA1->ISR)&0b10)) ;
			DMA1->IFCR |= 0b10;
			calc_adc_vals(sampleset);
			// Do time critical checking here:
			// the values are filtered enough to prevent spikes,
			// but the current measurement especially still has
			// too much noise for real logging.

			if(cur_state == STATE_CHA || cur_state == STATE_DSCH)
			{
				// Sudden voltage spike, due to battery disconnection etc.
				if((cur_state == STATE_CHA && v_sense_midflt[sampleset] > v_stop+500) ||
				   (cur_state == STATE_DSCH && v_sense_midflt[sampleset] < v_stop-500))
					error('v');

				if(v_direct > v_sense + V_DIRECT_MAX_DELTA)
					error('w');
			}

			if(v_direct > V_DIRECT_MAX)
				error('V');

			if(do_handle_message)
			{
				// Handle messages more quickly.
				// Will still have older data in the buffer
				// but it will behave like a ring buffer
				// thing.
				break;
			}


		}

		// After SECOND_FILT_LEN samples, average the averages (yo dawg)

		calc_second_filt();


		// You really need to wait for the acknowledgement message
		// because messages are handled here.
		if(do_handle_message)
		{
			handle_message();
			do_handle_message = 0;
		}

		if(cur_state == STATE_CHA)
			adjust_charge();
		else if(cur_state == STATE_DSCH)
			adjust_discharge();
		set_hw_curr_limits();
	}
}

