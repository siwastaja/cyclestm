CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mcpu=cortex-m0 -mthumb -Wall -fstack-usage
LDFLAGS = -mcpu=cortex-m0 -mthumb -nostartfiles -gc-sections
# -L/usr/local/stm32/arm-none-eabi/lib/thumb -lc

DEPS = main.h own_std.h
OBJ = stm32init.o main.o own_std.o

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^
	$(OBJCOPY) -Obinary main.elf main.bin
	$(SIZE) main.elf

flash: main.bin
	stm32sprog -b 115200 -vw main.bin

stack:
	cat *.su
