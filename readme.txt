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

STM32F030F4
4K SRAM
16K Flash
48 MHz
TSSOP20

 1 BOOT0      short to 3V3 during powerup to run programming bootloader
 2 PF0     DI Input overvoltage protection - high: input exceeded approx. 14 + 3.3/2 = 15.65V
 3 PF1    DIO unused - not connected
 4 NRST       reset - not connected
 5 VDDA       Analog 3V3
 6 PA0     AI ADC0 thermistor input
 7 PA1     AI ADC1 Sense Voltage measurement
 8 PA2     DO PWM_HI/HI_ENA buck (charge) enable
 9 PA3     DO PWM_LO/LO_ENA boost (discharge) enable
10 PA4     AI ADC4 Direct output voltage measurement (for protection, mostly)
11 PA5     AI ADC5 Iref reference voltage measurement (very close to 50%), subtract from Imeas to get accurate I.
12 PA6     AI ADC6 Imeas, current measurement
13 PA7     DO TIM3_CH2  PWM I-set, valid range 0..50%
14 PB1     DO TIM3_CH4  PWM I+set, valid range 50%..100%
15 VSS        GND
16 VDD        Digital 3V3
17 PA9    DIO I2C SCL, (UART TX when programming)
18 PA10   DIO I2C SDA, (UART RX when programming)
19 PA13    DO Green LED, active low
20 PA14    DO RED LED, active low

Programming
Short BOOT0 pin to 3V3 during powerup; the device will enter programming mode.
User code in other devices on the same bus will wait for some time at boot so that bus is floating;
programming uses the bus in 0V/3V3 RS232 mode.

Addresses
7-bit address is fixed during compilation time. Make sure every node is programmed with a different address.
Max modules per bus is about 12 or 24 anyway due to EMI reasons. Every bus has its own address space.

I2C communication
Slow communication, heavily RC filtered. Going for about 20 kHz first.

Commands
Multi-byte values little endian.
Voltages are signed int16, in mv
Currents are signed int16, in mA
Temperature is 16-bit ADC reading, uint16.

0x11 Report status
     Response:
     0x11 Status report
     1 byte status flags
         bit 7:
         bit 6:
         bit 5:
         bit 4: Has been in CV, but went back to CC
         bit 3: In CV
         bit 2..0: current state
     2 bytes voltage [0..6000]
     2 bytes current [-32000..32000]
     2 bytes temperature [0..65535]

0x12 Report extended status
     Response:
     0x12 Extended status report
     7 bytes normal status report
     4 bytes V&I setpoints
     14 bytes safety limits
     2 bytes timeout

0x21 Set V&I
     2 bytes: CV voltage setpoint [0..5000] default 3400
     2 bytes: CC current setpoint [-27000..27000] default 0

0x22 Set stop condition current
     2 bytes: Stop condition current [-27000..27000] default 0
              if 0: omit CV phase (stop when voltage reached)
              note the sign. Positive for charge, negative for discharge.

0x23 Set safety limits
     Note: current limits are peak currents in the inductor,
           unlike the setpoint which is average current (output current)
     2 bytes: min sense voltage [-1..5000] default -1
     2 bytes: max sense voltage [0..5000] default 4400
     2 bytes: max direct voltage [0..6000] default 5400
     2 bytes: min current [-32000..-1] default -30000
     2 bytes: max current [1..32000] default 30000
     2 bytes: min temperature [0..65535] default 0
     2 bytes: max temperature [0..65535] default 65535

0x24 Set timeout for next operation
     2 bytes: timeout in seconds [1..65535] default 65535 

0x31 Change state:
     0x00 OFF - stop charging or discharging
     0x01 CHARGE - start charging
          current setpoint must be positive before starting
     0x02 DISCHARGE - start discharging
          current setpoint must be negative before starting
     0x03 ERROR - stop charging or discharging. Require power-off.


Procedure:
1) Set parameters
2) Always query extended status and verify the parameters!
3) Start the cycle using 0x31 Change state
4) Query data by using Query status every second or so.
