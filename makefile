CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -I. -Os -fno-common -mcpu=cortex-m0 -mthumb -Wall -fstack-usage
LDFLAGS = -nostartfiles

DEPS = main.h
OBJ = stm32init.o main.o

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^ -L/usr/local/stm32/arm-none-eabi/lib/ -lc
	$(OBJCOPY) -Obinary main.elf main.bin
	$(SIZE) main.elf

flash: main.bin
	stm32sprog -vw main.bin
