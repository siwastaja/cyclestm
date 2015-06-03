IMPORTANT: Safety mechanisms not properly implemented yet; make sure there always
is a battery connected to both sides and that the 12V side does not go overvoltage 
(14V absolute max)

Bidirectional synchronous buck DC/DC converter controller

1 CPU per module
3 independent modules per PCB
4 PCBs per cube (approx. 12x12x12 cm with a 12cm fan)
12 modules per cube
All modules in shared UART (polled communication)
At least one cube shares the bus, but to make EMI things
easier, maybe one bus per cube (12 modules). This allows
more aggressive analog filtration (RC) without the problem of
excessive bus capacitance.

STM32F030F4 (4K SRAM, 16K Flash, 48 MHz, TSSOP20)

 1 BOOT0      short to 3V3 during powerup to run programming bootloader
 2 PF0     DI Input overvoltage protection - high: input exceeded approx. 14 + 3.3/2 = 15.65V
 3 PF1     DO FET driver power control - high: enable power
 4 NRST       reset - not connected
 5 VDDA       Analog 3V3
 6 PA0     AI ADC0 thermistor input
 7 PA1     AI ADC1 Sense Voltage measurement
 8 PA2     DO PWM_HI/HI_ENA buck (charge) enable
 9 PA3     DO PWM_LO/LO_ENA boost (discharge) enable
10 PA4     AI ADC4 Direct output voltage measurement (for protection, mostly)
11 PA5     AI ADC5 Iref reference voltage measurement (very close to 50%), subtract from Imeas to get accurate I.
12 PA6     AI ADC6 Imeas, current measurement
13 PA7     DO TIM3_CH2  PWM I-set, valid range maybe about 10..50%
14 PB1     DO TIM3_CH4  PWM I+set, valid range maybe about 50%..90%
15 VSS        GND
16 VDD        Digital 3V3
17 PA9    DIO UART TX
18 PA10   DIO UART RX
19 PA13    DO Green LED, active low
20 PA14    DO RED LED, active low

Programming
Short BOOT0 pin to 3V3 during powerup; the device will enter programming mode.
User code in other devices on the same bus will wait for some time at boot so that bus is floating.

Addresses
ASCII integer, for now 1...255

Messaging:

; parses any message and flushes the RX buffer.

Format:  @ID:command [parameter];

@1:OFF;          stop
@1:DSCH;         start discharge
@1:CHA;          start charge
@1:MEAS;         report latest measurements and state
@1:VERB;         same as above, with extra info

@1:SETV 2345;        set CV voltage to 2345 mV
@1:SETI 10000;       set CC current to +10000 mA
@1:SETVSTOP 2345;    set immediate stopping voltage to 2345 mV
@1:SETISTOP 5000;    set immediate stopping current to +5000 mA

@1:PAR;          print the actual settings


Currents less than 2000 mA may be unreliable for now (some modules seem to be better than others)

Negative currents for discharge, positive for charge.
(Wrong sign / out-of-range are checked by the CPU.)

Procedure:
1) Set parameters
2) Always query and verify the parameters!
3) Start the cycle using CHA or DSCH
4) Query data by using MEAS or VERB every second or as you like.
