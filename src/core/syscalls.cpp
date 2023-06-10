#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <time.h>

#include "fsl_lpuart.h"

extern "C" {

int __aeabi_atexit(__attribute__((unused)) void* object,
                   __attribute__((unused)) void (*destructor)(void*),
                   __attribute__((unused)) void* dso_handle) {
    return 0;
}

int _getpid(void) { return 1; }

int _kill(__attribute__((unused)) int pid, __attribute__((unused)) int sig) {
    errno = EINVAL;
    return -1;
}

extern const uint32_t _pvHeapLimit;

extern const uint32_t __heap_size;

extern uint32_t __end_of_heap;

extern const uint32_t __stack_base;

void print_system_report() {

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

void _exit(int status) {
    printf("[ ERROR ] Exiting with status %d\r\n", status);

    print_system_report();
    _kill(status, -1);

    while (1) {} /* Make sure we hang here */
}

void __assert_func(const char* file,
                   int line,
                   __attribute__((unused)) const char* func,
                   const char* failedexpr) {

    printf("[ ERROR ] Assert %s failed at %s:%d\r\n", failedexpr, file, line);

    _exit(1);

    // Silence no-return
    while (1) {}
}

int _read(__attribute__((unused)) int file,
          __attribute__((unused)) char* ptr,
          int len) {
    return len;
}

int _write(__attribute__((unused)) int file, char* ptr, int len) {
    LPUART_WriteBlocking(LPUART1, (const uint8_t*)ptr, (size_t)len);
    return len;
}

int _close(__attribute__((unused)) int file) { return -1; }

int _fstat(__attribute__((unused)) int file, struct stat* st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(__attribute__((unused)) int file) { return 1; }

int _lseek(__attribute__((unused)) int file,
           __attribute__((unused)) int ptr,
           __attribute__((unused)) int dir) {
    return 0;
}

int _open(__attribute__((unused)) char* path,
          __attribute__((unused)) int flags,
          ...) {

    // Pretend like we always fail
    return -1;
}

int _wait(__attribute__((unused)) int* status) {
    errno = ECHILD;
    return -1;
}

int _unlink(__attribute__((unused)) char* name) {
    errno = ENOENT;
    return -1;
}

int _times(__attribute__((unused)) struct tms* buf) { return -1; }

int _stat(__attribute__((unused)) char* file, struct stat* st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(__attribute__((unused)) char* old_data,
          __attribute__((unused)) char* new_data) {
    errno = EMLINK;
    return -1;
}

int _fork(void) {
    errno = EAGAIN;
    return -1;
}

int _execve(__attribute__((unused)) char* name,
            __attribute__((unused)) char** argv,
            __attribute__((unused)) char** env) {
    errno = ENOMEM;
    return -1;
}
}

/**
 * @brief This is the function that is called when an uncaught C++
 * exception is encountered. The default version within the C++
 * library prints the name of the uncaught exception, but to do so
 * it must de-mangle its name - which causes a large amount of code
 * to be pulled in. The below minimal implementation can reduce
 * code size noticeably. Note that this function should not return.
 */
namespace __gnu_cxx {
    void __verbose_terminate_handler() {
        printf("ERROR - Uncaught C++ exception, this should not happen as "
               "exceptions are not allowed. Exiting...\r\n");
        exit(1);
        while (1) {}
    }
}
