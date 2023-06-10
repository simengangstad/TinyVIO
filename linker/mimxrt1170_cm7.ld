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
 * The BOARD SDRAM follows the following memory map:
 *
 * Description          Start        End         Size
 * Bss, data            8000 0000    82FF FFFF   48 MB
 * Non-cachable M7      8300 0000    837F FFFF    8 MB
 * Non-cachable M4      8380 0000    83BF FFFF    4 MB
 * RPMSG SH MEM M4/M7   83C0 0000    83C1 FFFF    4 KB
 * Not allocated        83C2 0000    83FF FFFF    4 MB - 4 KB
 *
 * Note that the non-cachable regions are not shared between the
 * M7 core and the M4 core in order to prevent collisions.
 */

MEMORY
{
  BOARD_FLASH (rx)          : ORIGIN = 0x30000000, LENGTH = 0x00FE0000 /* 16MB - 128 KB */
  CORE1_IMAGE (rx)          : ORIGIN = 0x30FC0000, LENGTH = 0x00020000 /* 128KB */
  SRAM_DTC_CM7 (rwx)        : ORIGIN = 0x20000000, LENGTH = 0x00078000 /* 480KB */
  SRAM_ITC_CM7 (rwx)        : ORIGIN = 0x00000000, LENGTH = 0x00008000 /* 32KB */
  SRAM_OC (rwx)             : ORIGIN = 0x20240000, LENGTH = 0x00140000 /* 1.25MB, note that this is the M7 core's OCRAM, not including the M4 OCRAM backdoor */
  PRIMARY_DATA (rwx)        : ORIGIN = 0x80000000, LENGTH = 0x03000000 /* 48MB */
  NON_CACHEABLE (rwx)       : ORIGIN = 0x83000000, LENGTH = 0x00800000 /* 8MB */
  RPMSG_SH_MEM (rwx)        : ORIGIN = 0x83C00000, LENGTH = 0x00002000 /* 8KB */
}


__BASE_BOARD_FLASH = ORIGIN(BOARD_FLASH);
__TOP_BOARD_FLASH = ORIGIN(BOARD_FLASH) + LENGTH(BOARD_FLASH);
__SIZE_BOARD_FLASH = LENGTH(BOARD_FLASH);

__BASE_CORE1_IMAGE = ORIGIN(CORE1_IMAGE);

__BASE_SRAM_DTC_CM7 = ORIGIN(SRAM_DTC_CM7);
__TOP_SRAM_DTC_CM7 = ORIGIN(SRAM_DTC_CM7) + LENGTH(SRAM_DTC_CM7);
__SIZE_SRAM_DTC_CM7 = LENGTH(SRAM_DTC_CM7);

__BASE_SRAM_ITC_CM7 = ORIGIN(SRAM_ITC_CM7);
__TOP_SRAM_ITC_CM7 = ORIGIN(SRAM_ITC_CM7) + LENGTH(SRAM_ITC_CM7);
__SIZE_SRAM_ITC_CM7 = LENGTH(SRAM_ITC_CM7);

__BASE_SRAM_OC = ORIGIN(SRAM_OC);
__TOP_SRAM_OC = ORIGIN(SRAM_OC) + LENGTH(SRAM_OC);
__SIZE_SRAM_OC = LENGTH(SRAM_OC);

__BASE_PRIMARY_DATA = ORIGIN(PRIMARY_DATA);
__TOP_PRIMARY_DATA = ORIGIN(PRIMARY_DATA) + LENGTH(PRIMARY_DATA);
__SIZE_PRIMARY_DATA = LENGTH(PRIMARY_DATA);

__BASE_NON_CACHEABLE = ORIGIN(NON_CACHEABLE);
__TOP_NON_CACHEABLE = ORIGIN(NON_CACHEABLE) + LENGTH(NON_CACHEABLE);
__SIZE_NON_CACHEABLE = LENGTH(NON_CACHEABLE);

__BASE_RPMSG_SH_MEM = ORIGIN(RPMSG_SH_MEM);
__TOP_RPMSG_SH_MEM = ORIGIN(RPMSG_SH_MEM) + LENGTH(RPMSG_SH_MEM);
__SIZE_RPMSG_SH_MEM = LENGTH(RPMSG_SH_MEM);

ENTRY(ResetISR)

SECTIONS
{
     /* Image Vector Table and Boot Data for booting from external flash */
    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
        . = 0x400 ;
        __boot_hdr_start__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.conf))
        . = 0x1000 ;
        __boot_hdr_ivt_loadaddr__  =  ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.ivt))
        . = 0x1020 ;
        __boot_hdr_boot_data_loadaddr__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.boot_data))
        . = 0x1030 ;
        __boot_hdr_dcd_loadaddr__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.dcd_data))
        __boot_hdr_end__ = ABSOLUTE(.) ;
        . = 0x2000 ;
    } >BOARD_FLASH

    /* Place main text section in BOARD FLASH */
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
        LONG(LOADADDR(.data_SRAM_DTC_CM7));
        LONG(    ADDR(.data_SRAM_DTC_CM7));
        LONG(  SIZEOF(.data_SRAM_DTC_CM7));
        LONG(LOADADDR(.data_SRAM_ITC_CM7));
        LONG(    ADDR(.data_SRAM_ITC_CM7));
        LONG(  SIZEOF(.data_SRAM_ITC_CM7));
        LONG(LOADADDR(.data_SRAM_OC));
        LONG(    ADDR(.data_SRAM_OC));
        LONG(  SIZEOF(.data_SRAM_OC));
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
        LONG(    ADDR(.bss_SRAM_DTC_CM7));
        LONG(  SIZEOF(.bss_SRAM_DTC_CM7));
        LONG(    ADDR(.bss_SRAM_ITC_CM7));
        LONG(  SIZEOF(.bss_SRAM_ITC_CM7));
        LONG(    ADDR(.bss_SRAM_OC));
        LONG(  SIZEOF(.bss_SRAM_OC));
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
    } > BOARD_FLASH

    _etext = .;

    .core1_code :
    {
        . = ALIGN(4) ;
        KEEP (*(.core1_code))
        *(.core1_code*)
        . = ALIGN(4) ;
    } > CORE1_IMAGE


    /* ---------------------------- Data ---------------------------- */


    .data_SRAM_DTC_CM7 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_SRAM_DTC_CM7 = .) ;
        *(.ramfunc.$SRAM_DTC_CM7)
        KEEP(*(DataQuickAccess))
        *(.data.$SRAM_DTC_CM7)
        *(.data.$SRAM_DTC_CM7.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_SRAM_DTC_CM7 = .) ;
     } > SRAM_DTC_CM7 AT>BOARD_FLASH


    .data_SRAM_ITC_CM7 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_SRAM_ITC_CM7 = .) ;
        *(.ramfunc.$SRAM_ITC_CM7)
        KEEP(*(VectorTableRAM))
        KEEP(*(CodeQuickAccess))
        *(.data.$SRAM_ITC_CM7)
        *(.data.$SRAM_ITC_CM7.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_SRAM_ITC_CM7 = .) ;
     } > SRAM_ITC_CM7 AT>BOARD_FLASH

    .data_SRAM_OC : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_SRAM_OC = .) ;
        *(.ramfunc.$SRAM_OC)
        *(.data.$SRAM_OC)
        *(.data.$SRAM_OC.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_SRAM_OC = .) ;
     } > SRAM_OC AT>BOARD_FLASH

    .data_NON_CACHEABLE : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_NON_CACHEABLE = .) ;
        *(.ramfunc.$NON_CACHEABLE)
        *(NonCacheable.init)
        *(.data.$NON_CACHEABLE)
        *(.data.$NON_CACHEABLE.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_NON_CACHEABLE = .) ;
     } > NON_CACHEABLE AT>BOARD_FLASH

    .data_RPMSG_SH : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RPMSG_SH_MEM = .) ;
        *(.ramfunc.$RPMSG_SH_MEM)
        *(.data.$RPMSG_SH_MEM)
        *(.data.$RPMSG_SH_MEM.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RPMSG_SH_MEM = .) ;
     } > RPMSG_SH_MEM AT>BOARD_FLASH

    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > PRIMARY_DATA AT> PRIMARY_DATA

    /* Default DATA section placed in PRIMARY_DATA */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_DEFAULT = .) ;
       *(vtable)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_DEFAULT = .) ;
    } > PRIMARY_DATA AT>BOARD_FLASH



    /* ---------------------------- BSS ---------------------------- */

    .bss_SRAM_DTC_CM7 : ALIGN(4)
    {
       PROVIDE(__start_bss_SRAM_DTC_CM7 = .) ;
       *(.bss.$SRAM_DTC_CM7)
       *(.bss.$SRAM_DTC_CM7.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_SRAM_DTC_CM7 = .) ;
    } > SRAM_DTC_CM7 AT> SRAM_DTC_CM7

    .bss_SRAM_ITC_CM7 : ALIGN(4)
    {
       PROVIDE(__start_bss_SRAM_ITC_CM7 = .) ;
       *(.bss.$SRAM_ITC_CM7)
       *(.bss.$SRAM_ITC_CM7.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_SRAM_ITC_CM7 = .) ;
    } > SRAM_ITC_CM7 AT> SRAM_ITC_CM7

    .bss_SRAM_OC : ALIGN(4)
    {
       PROVIDE(__start_bss_SRAM_OC = .) ;
       *(.bss.$SRAM_OC)
       *(.bss.$SRAM_OC.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_SRAM_OC = .) ;
    } > SRAM_OC AT> SRAM_OC

    .bss_NON_CACHEABLE : ALIGN(4)
    {
       PROVIDE(__start_bss_NON_CACHEABLE = .) ;
       *(NonCacheable)
       *(.bss.$NON_CACHEABLE)
       *(.bss.$NON_CACHEABLE.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_NON_CACHEABLE = .) ;
    } > NON_CACHEABLE AT> NON_CACHEABLE

    .bss_RPMSG_SH : ALIGN(4)
    {
       PROVIDE(__start_bss_rpmsg_sh = .) ;
       *(.bss.$RPMSG_SH_MEM)
       *(.bss.$RPMSG_SH_MEM.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_rpmsg_sh = .) ;
    } > RPMSG_SH_MEM AT> RPMSG_SH_MEM

    /* Default BSS section placed in PRIMARY_DATA */
    .bss : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_DEFAULT = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_DEFAULT = .) ;
        PROVIDE(end = .);
    } > PRIMARY_DATA AT> PRIMARY_DATA


    /* ---------------------------- NOINIT ---------------------------- */


    .noinit_SRAM_DTC_CM7 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_SRAM_DTC_CM7 = .) ;
       *(.noinit.$SRAM_DTC_CM7)
       *(.noinit.$SRAM_DTC_CM7.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_SRAM_DTC_CM7 = .) ;
    } > SRAM_DTC_CM7 AT> SRAM_DTC_CM7

    .noinit_SRAM_ITC_CM7 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_SRAM_ITC_CM7 = .) ;
       *(.noinit.$SRAM_ITC_CM7)
       *(.noinit.$SRAM_ITC_CM7.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_SRAM_ITC_CM7 = .) ;
    } > SRAM_ITC_CM7 AT> SRAM_ITC_CM7

    .noinit_SRAM_OC (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_SRAM_OC = .) ;
       *(.noinit.$SRAM_OC)
       *(.noinit.$SRAM_OC.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_SRAM_OC = .) ;
    } > SRAM_OC AT> SRAM_OC

    .noinit_NON_CACHEABLE (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_NON_CACHEABLE = .) ;
       *(.noinit.$NON_CACHEABLE)
       *(.noinit.$NON_CACHEABLE.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_NON_CACHEABLE = .) ;
    } > NON_CACHEABLE AT> NON_CACHEABLE

    .noinit_RPMSG_SH (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_rpmsg_sh = .) ;
       *(.noinit.$RPMSG_SH_MEM)
       *(.noinit.$RPMSG_SH_MEM.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_rpmsg_sh = .) ;
    } > RPMSG_SH_MEM AT> RPMSG_SH_MEM

    /* Default NOINIT section placed in SRAM_DTC_CM7 */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_DEFAULT = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_DEFAULT = .) ;
    } > PRIMARY_DATA AT> PRIMARY_DATA

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > BOARD_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > BOARD_FLASH


    /* ---------------------------- Heap & Stack ---------------------------- */

    /* Reserve and place heap within memory map, placed in PRIMARY_DATA */
    __heap_size = 0x10000; /* 64KB */
    .heap :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += __heap_size;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > PRIMARY_DATA

     /* Reserve space in memory for Stack, placed in SRAM_DTC */
     __stack_size = 0x8000; /* 32KB */
    .stackfill :
    {
        . += __stack_size;
    } > SRAM_DTC_CM7


    /* Locate stack in memory map */
    .stack ORIGIN(SRAM_DTC_CM7) + LENGTH(SRAM_DTC_CM7) - __stack_size - 0 :  ALIGN(4)
    {
        __stack_base = .;
        . = ALIGN(4);
        __stack_top = . + __stack_size;
    } > SRAM_DTC_CM7

    /* ---------------------------- Image ---------------------------- */

    /**
     * Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;

    /**
     * Provide symbols for MIMXRT1160 parts for boot header generation code
     * to set image to be plain load image or XIP.
     * Config : Plain load image = false
     */
    __boot_load_address = ORIGIN(BOARD_FLASH);
    __boot_size = LENGTH(BOARD_FLASH);
}
