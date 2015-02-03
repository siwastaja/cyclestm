#ifndef  __MAIN_H
#define  __MAIN_H

int main(void) __attribute__((noreturn));

void uart_rx_handler() __attribute__((__interrupt__));
void uart_dma_finished_handler() __attribute__((__interrupt__));

void input_overvoltage_handler() __attribute__((__interrupt__));

#endif  // __MAIN_H 
