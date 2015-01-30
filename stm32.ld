MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
  stack(rwx): ORIGIN = 0x20000FFC, LENGTH = 0K

  rom (rx)  : ORIGIN = 0x00000000, LENGTH = 16K
}

SECTIONS
{
    .nvic_vector : 
    {
        *(vectors)    /* Vector table */
    } >rom

    .text : 
    {
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)        
    } >rom

    .data_init1 :
    ALIGN(4)
    {
        _DATAI_BEGIN = .;
    } >rom

    .data : 
    {
        _DATA_BEGIN = .;
        *(.data)      /* Data memory */
        *(.data.*)        
        _DATA_END = .;
    } >ram AT>rom

    .data_init2 : 
    {
        _DATAI_END = .;
    } >rom

    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)       /* Zero-filled run time allocate data memory */
        *(COMMON)        
        _BSS_END = .;
    } >ram 

    .heap :
    {
        _HEAP = .;
    } >ram

    .stack :
    {
        _STACKTOP = .;
    } >stack

}  
