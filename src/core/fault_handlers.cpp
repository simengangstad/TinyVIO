#include <stdio.h>
#include <stdlib.h>

#include "fsl_device_registers.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * One of the issues with applications that make use of semihosting operations
 * (such as printf calls) is that the code will not execute correctly when the
 * debugger is not connected. Generally this will show up with the application
 * appearing to just hang. This may include the application running from reset
 * or powering up the board (with the application already in FLASH), and also
 * as the application failing to continue to execute after a debug session is
 * terminated.
 *
 * The problem here is that the "bottom layer" of the semihosted variants of
 * the C library, semihosting is implemented by a "BKPT 0xAB" instruction.
 * When the debug tools are not connected, this instruction triggers a hard
 * fault - and the default hard fault handler within an application will
 * typically just contains an infinite loop - causing the application to
 * appear to have hang when no debugger is connected.
 *
 * The below code provides an example hard fault handler which instead looks
 * to see what the instruction that caused the hard fault was - and if it
 * was a "BKPT 0xAB", then it instead returns back to the user application.
 *
 * In most cases this will allow applications containing semihosting
 * operations to execute (to some degree) when the debugger is not connected.
 *
 * == NOTE ==
 *
 * Correct execution of the application containing semihosted operations
 * which are vectored onto this hard fault handler cannot be guaranteed. This
 * is because the handler may not return data or return codes that the higher
 * level C library code or application code expects. This hard fault handler
 * is meant as a development aid, and it is not recommended to leave
 * semihosted code in a production build of your application!
 *
 */
void HardFault_Handler(void) {
    __asm(".syntax unified\n"
          // Check which stack is in use
          "MOVS   R0, #4  \n"
          "MOV    R1, LR  \n"
          "TST    R0, R1  \n"
          "BEQ    _MSP    \n"
          "MRS    R0, PSP \n"
          "B  _process      \n"
          "_MSP:  \n"
          "MRS    R0, MSP \n"
          // Load the instruction that triggered hard fault
          "_process:     \n"
          "LDR    R1,[R0,#24] \n"
          "LDRH    R2,[r1] \n"
          // Semihosting instruction is "BKPT 0xAB" (0xBEAB)
          "LDR    R3,=0xBEAB \n"
          "CMP     R2,R3 \n"
          "BEQ    _semihost_return \n"
          // Wasn't semihosting instruction so enter infinite loop
          "B . \n"
          // Was semihosting instruction, so adjust location to
          // return to by 1 instruction (2 bytes), then exit function
          "_semihost_return: \n"
          "ADDS    R1,#2 \n"
          "STR    R1,[R0,#24] \n"
          // Set a return value from semihosting operation.
          // 32 is slightly arbitrary, but appears to allow most
          // C Library IO functions sitting on top of semihosting to
          // continue to operate to some degree
          "MOVS   R1,#32 \n"
          "STR R1,[ R0,#0 ] \n" // R0 is at location 0 on stack
                                // Return from hard fault handler to application
          "BX LR \n"
          ".syntax divided\n");
}

extern const uint32_t _pvHeapLimit;

extern const uint32_t __heap_size;

extern uint32_t __end_of_heap;

extern const uint32_t __stack_base;

static void print_system_report() {
    void* sp;
    __asm volatile("mov %0, sp" : "=r"(sp));

    int64_t stack_available = ((int64_t)((uint32_t)sp) -
                               (int64_t)(((uint32_t)&__stack_base)));

    printf("        Stack available: %lld\r\n", stack_available);

    int64_t heap_available = 0;

    if (__end_of_heap == 0) {
        heap_available = (uint32_t)&__heap_size;
    }

    heap_available = (int64_t)((uint32_t)&_pvHeapLimit) -
                     (int64_t)((uint32_t)__end_of_heap);

    printf("        Heap available: %lld\r\n", heap_available);
}

void NMI_Handler(void) {
    printf("[ ERROR ] Reached NMI handler\r\n");
    exit(1);
}

void MemManage_Handler(void) {
    printf("[ ERROR ] Reached memory management fault handler\r\n");
    exit(1);
}

void BusFault_Handler(void) {

    if (SCB->CFSR & SCB_CFSR_BFARVALID_Msk) {
        printf("[ ERROR ] Reached bus fault handler at address: %lx\r\n",
               SCB->BFAR);
    } else {
        printf("[ ERROR ] Reached bus fault handler\r\n");
    }

    printf("          -> BusFault during floating point lazy state "
           "preservation: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_LSPERR_Msk) >> SCB_CFSR_LSPERR_Pos);
    printf("          -> BusFault on stacking for exception entry: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_STKERR_Msk) >> SCB_CFSR_STKERR_Pos);
    printf("          -> BusFault on unstacking for a return from exception: "
           "%ld\r\n",
           (SCB->CFSR & SCB_CFSR_UNSTKERR_Msk) >> SCB_CFSR_UNSTKERR_Pos);
    printf("          -> Imprecise data bus error: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk) >> SCB_CFSR_IMPRECISERR_Pos);
    printf("          -> Precise data bus error: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_PRECISERR_Msk) >> SCB_CFSR_PRECISERR_Pos);
    printf("          -> Instruction bus error: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_IBUSERR_Msk) >> SCB_CFSR_IBUSERR_Pos);

    print_system_report();

    exit(1);
}

void UsageFault_Handler(void) {
    printf("[ ERROR ] Reached usage fault handler. Status:\r\n");

    printf("          -> Divide by zero: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_DIVBYZERO_Msk) >> SCB_CFSR_DIVBYZERO_Pos);
    printf("          -> Unaligned access: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_UNALIGNED_Msk) >> SCB_CFSR_UNALIGNED_Pos);
    printf("          -> Attempt to execute co-processor instruction: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_NOCP_Msk) >> SCB_CFSR_NOCP_Pos);
    printf("          -> Attempt to do an exception with a bad value in the "
           "EX_RETURN number: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_INVPC_Msk) >> SCB_CFSR_INVPC_Pos);
    printf("          -> Attempt to switch to an invalid state: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_INVSTATE_Msk) >> SCB_CFSR_INVSTATE_Pos);
    printf("          -> Attempt to execute undefined instruction: %ld\r\n",
           (SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk) >> SCB_CFSR_UNDEFINSTR_Pos);

    print_system_report();

    exit(1);
}

void SVC_Handler(void) {
    printf("[ERROR] Reached SVC handler\r\n");
    exit(1);
}

void DebugMon_Handler(void) {
    printf("[ ERROR ] Reached debug montior handler\r\n");
    exit(1);
}

void PendSV_Handler(void) {
    printf("[ERROR] Reached pend SV handler\r\n");
    exit(1);
}

void IntDefaultHandler(void) {
    printf("[ ERROR ] Reached interrupt not defined handler. Interrupt "
           "declaration is missing! \r\n");
    exit(1);
}

#if defined(__cplusplus)
}
#endif
