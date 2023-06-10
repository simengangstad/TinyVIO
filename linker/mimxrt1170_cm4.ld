GROUP (
  "../lib/nxp/lib/libgcc.a"
  "../lib/nxp/lib/libc.a"
  "../lib/nxp/lib/libstdc++.a"
  "../lib/nxp/lib/libm.a"
  "../lib/nxp/lib/libcr_newlib_none.a"
  "../lib/nxp/lib/crti.o"
  "../lib/nxp/lib/crtn.o"
  "../lib/nxp/lib/crtbegin.o"
  "../lib/nxp/lib/crtend.o"
)

/**
 * Note: non-cachable region is not shared between the M7 core and the M4 core.
 */

MEMORY
{
  SRAM_ITC_CM4  (rwx) : ORIGIN = 0x1ffe0000, LENGTH = 0x00020000 /* 128 KB */
  SRAM_DTC_CM4  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000 /* 128 KB */
  NON_CACHEABLE (rwx) : ORIGIN = 0x83800000, LENGTH = 0x00400000 /* 4MB */
  RPMSG_SH_MEM (rwx)  : ORIGIN = 0x83C00000, LENGTH = 0x00002000 /* 8KB */
}

__BASE_SRAM_ITC_CM4 = ORIGIN(SRAM_ITC_CM4);
__TOP_SRAM_ITC_CM4 = ORIGIN(SRAM_ITC_CM4) + LENGTH(SRAM_ITC_CM4);
__SIZE_SRAM_ITC_CM4 = LENGTH(SRAM_ITC_CM4);

__BASE_SRAM_DTC_CM4 = ORIGIN(SRAM_DTC_CM4);
__TOP_SRAM_DTC_CM4 = ORIGIN(SRAM_DTC_CM4) + LENGTH(SRAM_DTC_CM4);
__SIZE_SRAM_DTC_CM4 = LENGTH(SRAM_DTC_CM4);

__BASE_NON_CACHEABLE = ORIGIN(NON_CACHEABLE);
__TOP_NON_CACHEABLE = ORIGIN(NON_CACHEABLE) + LENGTH(NON_CACHEABLE);
__SIZE_NON_CACHEABLE = LENGTH(NON_CACHEABLE);

__BASE_RPMSG_SH_MEM = ORIGIN(RPMSG_SH_MEM);
__TOP_RPMSG_SH_MEM = ORIGIN(RPMSG_SH_MEM) + LENGTH(RPMSG_SH_MEM);
__SIZE_RPMSG_SH_MEM = LENGTH(RPMSG_SH_MEM);

ENTRY(ResetISR)

SECTIONS
{
     /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_SRAM_ITC_CM4));
        LONG(    ADDR(.data_SRAM_ITC_CM4));
        LONG(  SIZEOF(.data_SRAM_ITC_CM4));
        LONG(LOADADDR(.data_NON_CACHEABLE));
        LONG(    ADDR(.data_NON_CACHEABLE));
        LONG(  SIZEOF(.data_NON_CACHEABLE));
        LONG(LOADADDR(.data_RPMSG_SH));
        LONG(    ADDR(.data_RPMSG_SH));
        LONG(  SIZEOF(.data_RPMSG_SH));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_SRAM_ITC_CM4));
        LONG(  SIZEOF(.bss_SRAM_ITC_CM4));
        LONG(    ADDR(.bss_NON_CACHEABLE));
        LONG(  SIZEOF(.bss_NON_CACHEABLE));
        LONG(    ADDR(.bss_RPMSG_SH));
        LONG(  SIZEOF(.bss_RPMSG_SH));
       __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
            /* C++ constructors etc */
            . = ALIGN(4);
            KEEP(*(.init))

            . = ALIGN(4);
            __preinit_array_start = .;
            KEEP (*(.preinit_array))
            __preinit_array_end = .;

            . = ALIGN(4);
            __init_array_start = .;
            KEEP (*(SORT(.init_array.*)))
            KEEP (*(.init_array))
            __init_array_end = .;

            KEEP(*(.fini));

            . = ALIGN(4);
            KEEP (*crtbegin.o(.ctors))
            KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
            KEEP (*(SORT(.ctors.*)))
            KEEP (*crtend.o(.ctors))

            . = ALIGN(4);
            KEEP (*crtbegin.o(.dtors))
            KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
            KEEP (*(SORT(.dtors.*)))
            KEEP (*crtend.o(.dtors))
            . = ALIGN(4);
            /* End C++ */
    } > SRAM_ITC_CM4

    _etext = .;

    /* ---------------------------- Data ---------------------------- */

    .data_SRAM_ITC_CM4 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_SRAM_ITC_CM4 = .) ;
        *(.ramfunc.$SRAM_ITC_CM4)
        *(.data.$SRAM_ITC_CM4)
        *(.data.$SRAM_ITC_CM4.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_SRAM_ITC_CM4 = .) ;
     } > SRAM_ITC_CM4 AT>SRAM_ITC_CM4

    .data_NON_CACHEABLE : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_NON_CACHEABLE = .) ;
        *(.ramfunc.$NON_CACHEABLE)
        *(.data.$NON_CACHEABLE)
        *(.data.$NON_CACHEABLE.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_NON_CACHEABLE = .) ;
     } > NON_CACHEABLE AT>SRAM_ITC_CM4

    .data_RPMSG_SH : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RPMSG_SH_MEM = .) ;
        *(.ramfunc.$RPMSG_SH_MEM)
        *(.data.$RPMSG_SH_MEM)
        *(.data.$RPMSG_SH_MEM.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RPMSG_SH_MEM = .) ;
     } > RPMSG_SH_MEM AT>SRAM_ITC_CM4

    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > SRAM_DTC_CM4 AT> SRAM_ITC_CM4

    /* Default DATA section placed in SRAM_DTC_CM4 */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_SRAM_DTC_CM4 = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_SRAM_ITC_CM4 = .) ;
    } > SRAM_DTC_CM4 AT>SRAM_ITC_CM4

    /* ---------------------------- BSS ---------------------------- */

    .bss_SRAM_ITC_CM4 : ALIGN(4)
    {
       PROVIDE(__start_bss_SRAM_ITC_CM4 = .) ;
       *(.bss.$SRAM_ITC_CM4)
       *(.bss.$SRAM_ITC_CM4.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_SRAM_ITC_CM4 = .) ;
    } > SRAM_ITC_CM4 AT> SRAM_ITC_CM4

    .bss_NON_CACHEABLE : ALIGN(4)
    {
       PROVIDE(__start_bss_NON_CACHEABLE = .) ;
       *(.bss.$NON_CACHEABLE)
       *(.bss.$NON_CACHEABLE.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_NON_CACHEABLE = .) ;
    } > NON_CACHEABLE AT> NON_CACHEABLE

    .bss_RPMSG_SH : ALIGN(4)
    {
       PROVIDE(__start_base_RPMSG_SH = .) ;
       *(.bss.$RPMSG_SH_MEM)
       *(.bss.$RPMSG_SH_MEM.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_base_RPMSG_SH = .) ;
    } > RPMSG_SH_MEM AT> RPMSG_SH_MEM

    /* Default BSS section placed in SRAM_DTC_cm4 */
    .bss : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_SRAM_DTC_CM4 = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_SRAM_DTC_CM4 = .) ;
        PROVIDE(end = .);
    } > SRAM_DTC_CM4 AT> SRAM_DTC_CM4

    /* ---------------------------- NOINIT ---------------------------- */

    /* NOINIT section for DTCM */
    .noinit_SRAM_ITC_CM4 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_SRAM_ITC_CM4 = .) ;
       *(.noinit.$SRAM_ITC_CM4)
       *(.noinit.$SRAM_ITC_CM4.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_SRAM_ITC_CM4 = .) ;
    } > SRAM_ITC_CM4 AT> SRAM_ITC_CM4

    .noinit_NON_CACHEABLE (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_NON_CACHEABLE = .) ;
       *(.noinit.$NON_CACHEABLE)
       *(.noinit.$NON_CACHEABLE.*)
       *(NonCacheable)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_NON_CACHEABLE = .) ;
    } > NON_CACHEABLE AT> NON_CACHEABLE

    .noinit_RPMSG_SH (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RPMSG_SH_MEM = .) ;
       *(.noinit.$RPMSG_SH_MEM)
       *(.noinit.$RPMSG_SH_MEM.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RPMSG_SH_MEM = .) ;
    } > RPMSG_SH_MEM AT> RPMSG_SH_MEM

    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_SRAM_DTC_CM4 = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_SRAM_DTC_CM4 = .) ;
    } > SRAM_DTC_CM4 AT> SRAM_DTC_CM4

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > SRAM_ITC_CM4

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > SRAM_ITC_CM4

    /* ---------------------------- Heap & Stack ---------------------------- */

    /* Reserve and place Heap within memory map */
    __heap_size = 0x2000;
    .heap :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += __heap_size;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM_DTC_CM4

     /* Reserve space in memory for Stack */
     __stack_size = 0x4000;
    .heap2stackfill  :
    {
        . += __stack_size;
    } > SRAM_DTC_CM4

    /**
     * Locate actual Stack in memory map. Stack is located at the top
     * of DTCM - stack size and grows up to the end of ITCM.
     * */
    .stack ORIGIN(SRAM_DTC_CM4) + LENGTH(SRAM_DTC_CM4) - __stack_size - 0:  ALIGN(4)
    {
        __stack_base = .;
        . = ALIGN(4);
        __stack_top = . + __stack_size;
    } > SRAM_DTC_CM4

    /* ---------------------------- Image ---------------------------- */

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;

    /* Provide symbols for MIMXRT1160 parts for boot header generation code
     * to set image to be plain load image or XIP.
     * Config : Plain load image = false
     */
    __boot_load_address = ORIGIN(SRAM_ITC_CM4);
    __boot_size = LENGTH(SRAM_ITC_CM4);
}
