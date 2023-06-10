#ifndef MEMORY_H_
#define MEMORY_H_

#include <stdint.h>

namespace memory {
    /**
     * @brief Sets up the MPU with the rules for access the various different
     * memory areas as well as the cache policy.
     */
    void configure_access_policy();

    /**
     * @brief Return the amount of heap used. Will use the heap pointer, so does
     * not provide an exact value, but the top value of the heap.
     */
    int64_t current_heap_size();

    /**
     * @return Currently available memory on heap. Will use the heap pointer,
     * thus this number will not represent the exact amount of heap available,
     * but an upper boundary.
     */
    int64_t available_on_heap();

    /**
     * @return The number of bytes currently used on the stack.
     */
    int64_t current_stack_size();

    /**
     * @return Currently available memory on stack. If the stack has overflown,
     * a negative number is returned.
     */
    int64_t available_on_stack();
}

#endif
