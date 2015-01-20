Bidirectional synchronous buck DC/DC converter controller

1 CPU per module
3 independent modules per PCB
4 PCBs per cube (approx. 12x12x12 cm with a 12cm fan)
12 modules per cube
All modules in I2C bus
At least one cube shares the bus, but to make EMI things
easier, maybe one bus per cube (12 modules). This allows
more aggressive analog filtration (RC) without the problem of
excessive bus capacitance.

STM32F030
4K SRAM
16K Flash
