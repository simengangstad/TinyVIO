#include "memory.h"

#include "fsl_device_registers.h"

#include <assert.h>
#include <stdlib.h>

#if __CORTEX_M == 7

static void mpu_init() {

    extern uint32_t __BASE_RPMSG_SH_MEM;
    extern uint32_t __TOP_RPMSG_SH_MEM;
    uint32_t rpmsg_memory_area_start = (uint32_t)(&__BASE_RPMSG_SH_MEM);
    uint32_t rpmsg_memory_area_size  = (uint32_t)(&__TOP_RPMSG_SH_MEM) -
                                      rpmsg_memory_area_start;

    extern uint32_t __BASE_NON_CACHEABLE;
    extern uint32_t __TOP_NON_CACHEABLE;
    uint32_t non_cache_memory_area_start = (uint32_t)(&__BASE_NON_CACHEABLE);
    uint32_t non_cache_memory_area_size  = (uint32_t)(&__TOP_NON_CACHEABLE) -
                                          non_cache_memory_area_start;

#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
    /* Disable I cache and D cache */
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR)) {
        SCB_DisableICache();
    }
#endif

#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR)) {
        SCB_DisableDCache();
    }
#endif

    ARM_MPU_Disable();

    // clang-format off
    /**
     * MPU configure:
     *
     * ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable, SubRegionDisable, Size) 
     *
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches disabled.
     *
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode. 
     *                         Use MACROS defined in mpu_armv7.h:
     *                         ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     *
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribute    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable
     *     0             0           1           0              Normal             not shareable   Outer and inner write through no write allocate
     *     0             0           1           1              Normal             not shareable   Outer and inner write back no write allocate
     *     0             1           1           0              Normal             shareable       Outer and inner write through no write allocate
     *     0             1           1           1              Normal             shareable       Outer and inner write back no write allocate
     *     1             0           0           0              Normal             not shareable   outer and inner noncache
     *     1             1           0           0              Normal             shareable       outer and inner noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write back write/read allocate
     *     1             1           1           1              Normal             shareable       outer and inner write back write/read allocate
     *     2             x           0           0              Device             not shareable
     *
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     *
     * param Size              Region size of the region to be configured: ARM_MPU_REGION_SIZE_xxx MACRO.
     */

    // clang-format on

    // In the following definitions, some areas are overlapping. However, the
    // sub-regions will overwrite the respective configurations

    /* Region 0 setting: Instruction access disabled, No data access
     * permission.
     *
     * Add default region to deny access to whole address space to
     * workaround speculative prefetch. See Arm errata 1013783-B for
     * more details.
     */
    MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(1,
                             ARM_MPU_AP_NONE,
                             0,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_4GB);

    // ----------------------------- SEMC -----------------------------
    //
    // This is the main region for e.g. SDRAM and other memories under
    // the SEMC umbrella

    /**
     * Region 1 setting: Memory with Device type, not shareable,
     * non-cacheable.
     *
     * This is the main block for all the SEMC address space.
     */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_512MB);

    // ----------------------------- FlexSPI -----------------------------

    /**
     * Region 2 setting, for FlexSPI: Memory with Device type, not shareable,
     * non-cacheable.
     */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x60000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_512MB);

    // ------------------------ Lower memory region ----------------------

    /**
     * Region 3 setting, up to AIPS-1: Memory with Device type, not shareable,
     * non-cacheable.
     *
     * This is thus from 0000 0000 -> 4000 0000.
     */
    MPU->RBAR = ARM_MPU_RBAR(3, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_1GB);

    // ----------------------------- TCM -----------------------------

    /**
     * Region 4 setting, for ITCM: Memory with Normal type, not shareable,
     * outer/inner write back
     *
     * Note here that the ITCM is only 32 KB, which is what is allocated for it
     * during start-up.
     */
    MPU->RBAR = ARM_MPU_RBAR(4, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             1,
                             0,
                             ARM_MPU_REGION_SIZE_32KB);

    /**
     * Region 5 setting, for DTCM: Memory with Normal type, not shareable,
     * outer/inner write back
     *
     * The DTCM has a size of 480 KB, but we can only define sizes 2^n, so we
     * have to declare 512 KB here.
     */
    MPU->RBAR = ARM_MPU_RBAR(5, 0x20000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             1,
                             0,
                             ARM_MPU_REGION_SIZE_512KB);

    // ----------------------------- OCRAM -----------------------------

    // The following sets the policy for the OCRAM. The OCRAM is in total 1.5
    // MB (including the FlexRAM, which includes DTCM and ITCM, however the
    // policies for these regions will override the rules due to being smaller
    // in size, and we map them to a different address space anyway).
    //
    // clang-format off
    //
    // 1. 2020 0000 -> 2030 0000 (1 MB, OCRAM M4, OCRAM1, OCRAM2, OCRAM1 ECC, OCRAM2 ECC, OCRAM M7 FlexRAM ECC)
    // 2. 2030 0000 -> 2038 0000 (512 KB, OCRAM M7 FlexRAM)
    //
    // clang-format on

#if defined(CACHE_MODE_WRITE_THROUGH) && CACHE_MODE_WRITE_THROUGH
    /**
     * Region 6 setting, for OCRAM lower-part: Memory with Normal type, not
     * shareable, write through
     */
    MPU->RBAR = ARM_MPU_RBAR(6, 0x20200000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_1MB);
    /**
     * Region 7 setting, for OCRAM upper-part: Memory with Normal type, not
     * shareable, write trough
     */
    MPU->RBAR = ARM_MPU_RBAR(7, 0x20300000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_512KB);
#else
    /**
     * Region 6 setting, for OCRAM lower-part: Memory with Normal type, not
     * shareable, outer/inner write back
     */
    MPU->RBAR = ARM_MPU_RBAR(6, 0x20200000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             1,
                             0,
                             ARM_MPU_REGION_SIZE_1MB);

    /**
     * Region 7 setting, for OCRAM upper-part: Memory with Normal type, not
     * shareable, outer/inner write back
     */
    MPU->RBAR = ARM_MPU_RBAR(7, 0x20300000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             1,
                             0,
                             ARM_MPU_REGION_SIZE_512KB);
#endif

    // ----------------------------- FLASH -----------------------------
    //
    // Region for external program memory

#if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
    /**
     * Region 8 setting, for FLASH: Memory with Normal type, not shareable,
     * outer/inner write back.
     */
    MPU->RBAR = ARM_MPU_RBAR(8, 0x30000000U);
    MPU->RASR =
        ARM_MPU_RASR(0, ARM_MPU_AP_RO, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_16MB);
#endif

    // ----------------------------- SDRAM -----------------------------

#ifdef USE_SDRAM

#if defined(CACHE_MODE_WRITE_THROUGH) && CACHE_MODE_WRITE_THROUGH
    /**
     * Region 9 setting, for SDRAM: Memory with Normal type, not shareable,
     * write trough
     */
    MPU->RBAR = ARM_MPU_RBAR(9, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_64MB);
#else
    /**
     * Region 9 setting, for SDRAM: Memory with Normal type, not shareable,
     * outer/inner write back
     */
    MPU->RBAR = ARM_MPU_RBAR(9, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             1,
                             0,
                             ARM_MPU_REGION_SIZE_64MB);
#endif
#endif

    // ----------------------------- RPMSG -----------------------------

#if defined(__USE_SHMEM)
    volatile uint32_t i = 0;
    while ((rpmsg_memory_area_size >> i) > 0x1U) { i++; }

    if (i != 0) {
        /* The MPU region size should be 2^N, 5<=N<=32, region base should
         * be multiples of size. */
        assert(!(rpmsg_memory_area_start % rpmsg_memory_area_size));
        assert(rpmsg_memory_area_size == (uint32_t)(1 << i));
        assert(i >= 5);

        /* Region 10 setting: Memory with Normal type, not shareable,
         * non-cacheable */
        MPU->RBAR = ARM_MPU_RBAR(10, rpmsg_memory_area_start);
        MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, i - 1);
    }
#endif

    // ------------------------- Non-cacheable -------------------------

    i = 0;
    while ((non_cache_memory_area_size >> i) > 0x1U) { i++; }

    if (i != 0) {
        /* The MPU region size should be 2^N, 5<=N<=32, region base should
         * be multiples of size. */
        assert(!(non_cache_memory_area_start % non_cache_memory_area_size));
        assert(non_cache_memory_area_size == (uint32_t)(1 << i));
        assert(i >= 5);

        /* Region 10 setting: Memory with Normal type, not shareable,
         * non-cacheable */
        MPU->RBAR = ARM_MPU_RBAR(11, non_cache_memory_area_start);
        MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, i - 1);
    }

    // ------------------------------ Misc ----------------------------

    /**
     * Region 11 setting, for AIPS: Memory with Device type, not shareable,
     * non-cacheable
     */
    MPU->RBAR = ARM_MPU_RBAR(12, 0x40000000);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_16MB);

    /**
     * Region 12 setting, for AIPS-1: Memory with Device type, not shareable,
     * non-cacheable
     */
    MPU->RBAR = ARM_MPU_RBAR(13, 0x41000000);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_2MB);

    /**
     * Region 13 setting, for AIPS-1: Memory with Device type, not shareable,
     * non-cacheable
     */
    MPU->RBAR = ARM_MPU_RBAR(14, 0x41400000);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_1MB);

    /**
     * Region 14 setting, for AIPS-1: Memory with Device type, not shareable,
     * non-cacheable
     */
    MPU->RBAR = ARM_MPU_RBAR(15, 0x41800000);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_2MB);

    /**
     * Region 15 setting, for AIPS-1: Memory with Device type, not shareable,
     * non-cacheable
     */
    MPU->RBAR = ARM_MPU_RBAR(16, 0x42000000);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             2,
                             0,
                             0,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_1MB);

    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    SCB_EnableDCache();
#endif

#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
    SCB_EnableICache();
#endif
}

#elif __CORTEX_M == 4

void mpu_init() {

#if defined(__USE_SHMEM)
    extern uint32_t __BASE_RPMSG_SH_MEM;
    extern uint32_t __TOP_RPMSG_SH_MEM;
    uint32_t rpmsg_memory_area_start = (uint32_t)(&__BASE_RPMSG_SH_MEM);
    uint32_t rpmsg_memory_area_size  = (uint32_t)(&__TOP_RPMSG_SH_MEM) -
                                      rpmsg_memory_area_start;

    extern uint32_t __BASE_NON_CACHEABLE;
    extern uint32_t __TOP_NON_CACHEABLE;
    uint32_t non_cache_memory_area_start = (uint32_t)(&__BASE_NON_CACHEABLE);
    uint32_t non_cache_memory_area_size  = (uint32_t)(&__TOP_NON_CACHEABLE) -
                                          non_cache_memory_area_start;

    /* Only config non-cacheable region on system bus */
    assert((uint32_t)(&__BASE_RPMSG_SH_MEM) >= 0x20000000);
#endif

    /* Disable code bus cache */
    if (LMEM_PCCCR_ENCACHE_MASK == (LMEM_PCCCR_ENCACHE_MASK & LMEM->PCCCR)) {
        /* Enable the processor code bus to push all modified lines. */
        LMEM->PCCCR |= LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK |
                       LMEM_PCCCR_GO_MASK;
        /* Wait until the cache command completes. */
        while ((LMEM->PCCCR & LMEM_PCCCR_GO_MASK) != 0U) {}
        /* As a precaution clear the bits to avoid inadvertently re-running
         * this command. */
        LMEM->PCCCR &= ~(LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK);
        /* Now disable the cache. */
        LMEM->PCCCR &= ~LMEM_PCCCR_ENCACHE_MASK;
    }

    /* Disable system bus cache */
    if (LMEM_PSCCR_ENCACHE_MASK == (LMEM_PSCCR_ENCACHE_MASK & LMEM->PSCCR)) {
        /* Enable the processor system bus to push all modified lines. */
        LMEM->PSCCR |= LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK |
                       LMEM_PSCCR_GO_MASK;
        /* Wait until the cache command completes. */
        while ((LMEM->PSCCR & LMEM_PSCCR_GO_MASK) != 0U) {}
        /* As a precaution clear the bits to avoid inadvertently re-running
         * this command. */
        LMEM->PSCCR &= ~(LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK);
        /* Now disable the cache. */
        LMEM->PSCCR &= ~LMEM_PSCCR_ENCACHE_MASK;
    }

    ARM_MPU_Disable();

    // --------------------------- OCRAM -----------------------------

    /**
     * Region 0 setting, for OCRAM lower part: Memory with Normal type, not
     * shareable, write trough
     */
    MPU->RBAR = ARM_MPU_RBAR(0, 0x20200000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_1MB);

    /**
     * Region 1 setting, for OCRAM upper-part: Memory with Normal type, not
     * shareable, write through
     */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x20300000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_512KB);

    // ----------------------------- SDRAM -----------------------------

    /**
     * Region 2 setting: Memory with Normal type, not shareable, write through
     */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0,
                             ARM_MPU_AP_FULL,
                             0,
                             0,
                             1,
                             0,
                             0,
                             ARM_MPU_REGION_SIZE_64MB);

    // ----------------------------- RPMSG -----------------------------

#if defined(__USE_SHMEM)
    uint32_t i = 0;

    while ((rpmsg_memory_area_size >> i) > 0x1U) { i++; }

    if (i != 0) {
        /* The MPU region size should be 2^N, 5<=N<=32, region base should
         * be multiples of size. */
        assert(!(rpmsg_memory_area_start % rpmsg_memory_area_size));
        assert(rpmsg_memory_area_size == (uint32_t)(1 << i));
        assert(i >= 5);

        /* Region 1 setting: Memory with device type, not shareable,
         * non-cacheable */
        MPU->RBAR = ARM_MPU_RBAR(3, rpmsg_memory_area_start);
        MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, i - 1);
    }
#endif

    // ------------------------- Non-cacheable -------------------------

    i = 0;
    while ((non_cache_memory_area_size >> i) > 0x1U) { i++; }

    if (i != 0) {
        // The MPU region size should be 2^N, 5<=N<=32, region base should
        // be multiples of size.
        assert(!(non_cache_memory_area_start % non_cache_memory_area_size));
        assert(non_cache_memory_area_size == (uint32_t)(1 << i));
        assert(i >= 5);

        // Region 1 setting: Memory with device type, not shareable,
        // non-cacheable
        MPU->RBAR = ARM_MPU_RBAR(4, non_cache_memory_area_start);
        MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, i - 1);
    }

    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enables the processor system bus to invalidate all lines in both
    ways. and Initiate the processor system bus cache command. */
    LMEM->PSCCR |= LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK |
                   LMEM_PSCCR_GO_MASK;
    /* Wait until the cache command completes */
    while ((LMEM->PSCCR & LMEM_PSCCR_GO_MASK) != 0U) {}
    /* As a precaution clear the bits to avoid inadvertently re-running this
     * command. */
    LMEM->PSCCR &= ~(LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK);
    /* Now enable the system bus cache. */
    LMEM->PSCCR |= LMEM_PSCCR_ENCACHE_MASK;

    /* Enables the processor code bus to invalidate all lines in both ways.
    and Initiate the processor code bus code cache command. */
    LMEM->PCCCR |= LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK |
                   LMEM_PCCCR_GO_MASK;
    /* Wait until the cache command completes. */
    while ((LMEM->PCCCR & LMEM_PCCCR_GO_MASK) != 0U) {}
    /* As a precaution clear the bits to avoid inadvertently re-running this
     * command. */
    LMEM->PCCCR &= ~(LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK);
    /* Now enable the code bus cache. */
    LMEM->PCCCR |= LMEM_PCCCR_ENCACHE_MASK;
}
#endif

/**
 * @brief The address of the limit of the heap. Defined in linker file.
 */
extern const uint32_t _pvHeapLimit;

/**
 * @brief The start address of the heap. Defined in the linker file.
 */
extern const uint32_t _pvHeapStart;

/**
 * @brief The size of the heap. Defined in the linker file.
 */
extern const uint32_t __heap_size;

/**
 * @brief Address pointing to the end of the current heap (grows with new
 * allocations).
 */
extern uint32_t __end_of_heap;

/**
 * @brief Bottom address of the stack. Defined in the linker file.
 */
extern const uint32_t __stack_base;

/**
 * @brief Top address of the stack. Defined in the linker file.
 */
extern const uint32_t __stack_top;

namespace memory {
    void configure_access_policy() { mpu_init(); }

    int64_t current_heap_size() {
        if (__end_of_heap == 0) {
            return 0;
        }

        return (int64_t)((uint32_t)__end_of_heap) -
               (int64_t)((uint32_t)&_pvHeapStart);
    }

    int64_t available_on_heap() {
        if (__end_of_heap == 0) {
            return (uint32_t)&__heap_size;
        }

        return (int64_t)((uint32_t)&_pvHeapLimit) -
               (int64_t)((uint32_t)__end_of_heap);
    }

    int64_t current_stack_size() {

        // Note that the stack grows downwards, so the used bytes will be the
        // top of the stack minus the current stack pointer
        void* sp;
        __asm volatile("mov %0, sp" : "=r"(sp));

        const int64_t used = ((int64_t)((uint32_t)&__stack_top) -
                              (int64_t)(((uint32_t)sp)));

        return used;
    }

    int64_t available_on_stack() {

        // Note that the stack grows downwards, so the available bytes will be
        // the stack pointer minus the bottom of the stack
        void* sp;
        __asm volatile("mov %0, sp" : "=r"(sp));

        const int64_t available = ((int64_t)((uint32_t)sp) -
                                   (int64_t)(((uint32_t)&__stack_base)));

        return available;
    }
}
