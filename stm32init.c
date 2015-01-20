#include <stdint.h>

#include "main.h"

void stm32init(void);
void nmi_handler(void);
void hardfault_handler(void);

extern unsigned int _STACKTOP;

// Define the vector table
unsigned int * the_nvic_vector[4] 
__attribute__ ((section(".nvic_vector")))= {
    (unsigned int *)	&_STACKTOP,        // stack pointer
    (unsigned int *) 	stm32init,    // code entry point
    (unsigned int *)	nmi_handler,       // NMI handler (not really)
    (unsigned int *)	hardfault_handler  // hard fault handler (let's hope not)
};


extern unsigned int _BSS_BEGIN;
extern unsigned int _BSS_END;

extern unsigned int _DATA_BEGIN;
extern unsigned int _DATA_END;
extern unsigned int _DATAI_BEGIN;
extern unsigned int _DATAI_END;

void stm32init(void)
{
    uint32_t* bss_begin = (uint32_t*)&_BSS_BEGIN;
    uint32_t* bss_end   = (uint32_t*)&_BSS_END;
    while(bss_begin < bss_end)
    {
        *bss_begin = 0;
        bss_begin++;
    }

    uint32_t* data_begin  = (uint32_t*)&_DATA_BEGIN;
    uint32_t* data_end    = (uint32_t*)&_DATA_END;
    uint32_t* datai_begin = (uint32_t*)&_DATAI_BEGIN;
    uint32_t* datai_end   = (uint32_t*)&_DATAI_END;

    uint32_t data_size  = data_end  - data_begin;
    uint32_t datai_size = datai_end - datai_begin;

    if(data_size != datai_size) {
        //Linker script is not correct.
        while(1);
    }

    while(data_begin < data_end)
    {
        *data_begin = *datai_begin;
        data_begin++;
        datai_begin++;
    }


    main();
}


void nmi_handler(void)
{
    return ;
}

void hardfault_handler(void)
{
    return ;
}
