#ifndef  __MAIN_H
#define  __MAIN_H

int main(void) __attribute__((noreturn));

void uart_rx_handler() __attribute__((__interrupt__));

#endif  // __MAIN_H 
