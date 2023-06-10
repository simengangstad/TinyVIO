#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef DEBUG
#include <assert.h>
#include <stdio.h>
#endif

/**
 * @brief Custom allocator which uses a heap to store data. The allocator is
 * split between two memory regions, where the second buffer acts as a
 * backup.
 */
class Allocator {

  public:
    /**
     * @brief Allocated block of memory with metadata and pointer to the
     * actual data. Note that the data in the block is aligned with 32-bit
     * architecture.
     */
    struct Block {

        /**
         * @brief Block size, encodes size and whether the block is used.
         */
        size_t size;

        /**
         * @brief Whether this block is currently used.
         */
        bool used;

        /**
         * @brief Pointer to the previous block in the list.
         */
        Block* previous;

        /**
         * @brief Pointer to the next block in the list.
         */
        Block* next;

        /**
         * @brief Pointer to the data.
         */
        uint8_t data[1];
    };

  public:
    /**
     * @brief Size of the header of the block.
     */
    constexpr static size_t BLOCK_HEADER_SIZE = sizeof(Block) -
                                                sizeof(uint8_t*);

    /**
     * @brief Constructs an allocator where the prioritised @p
     * primary_buffer is passed. This buffer will be used for all
     * allocations which fit. The fallback is the @p secondary_buffer.
     *
     * @param primary_buffer_ Pointer to the primary buffer, which will have
     * the highest priority for allocations.
     * @param primary_buffer_size_ Size of the primary buffer.
     * @param secondary_buffer_ Pointer to the secondary buffer, which will
     * be used as a back-up if the primary buffer is too small for an
     * allocation.
     * @param secondary_buffer_size_ Size of the secondary buffer
     */
    Allocator(uint8_t* primary_buffer_,
              const size_t primary_buffer_size_,
              uint8_t* secondary_buffer_,
              const size_t secondary_buffer_size_)
        : primary_buffer(primary_buffer_),
          primary_buffer_size(primary_buffer_size_),
          secondary_buffer(secondary_buffer_),
          secondary_buffer_size(secondary_buffer_size_) {

        reset();
    }

    /**
     * @brief Reserves space in the allocator for a of size @p
     * rows times @p columns. The returned buffer will point to memory in
     * either the allocator's primary or secondary buffer, depending on
     * whether the primary buffer has enough capacity or not.
     *
     * @param size The number of bytes to allocate.
     */
    inline void* allocate(const size_t size) {
        Block* block = first_fit(size);

        if (block == nullptr) {
            return nullptr;
        }

        if (can_split_block(block, size)) {
            block       = split(block, size);
            block->size = size;
        }

        block->used = true;

#ifdef DEBUG
        assert(check_consistency());
#endif

        return (void*)block->data;
    }

    /**
     * @brief Reallocates @p previous for the new size @p size
     *
     * @param previous [in] The previous data.
     * @param size [in] The new size.
     */
    inline void* reallocate(void* previous, const size_t size) {

        const Block* previous_block = get_block(previous);

        // If the reallocation fits within the previous allocation, we just
        // reuse it
        if (size < previous_block->size) {
            return (void*)previous_block->data;
        }

        void* new_allocation = allocate(size);

        if (new_allocation == nullptr) {
            this->free(previous);
            return nullptr;
        }

        memcpy(new_allocation, previous, previous_block->size);

        this->free(previous);

#ifdef DEBUG
        assert(check_consistency());
#endif

        return new_allocation;
    }

    /**
     * @brief Frees up the space occupied by @p data.
     *
     * @param data The data to free.
     */
    inline void free(void* data) {
        if (data == nullptr) {
            return;
        }

        Block* block = get_block(data);

        if (can_merge_block(block)) {
            block = merge_block(block);
        }

        block->used = false;

#ifdef DEBUG
        assert(check_consistency());
#endif
    }

    /**
     * @return Number of bytes available for allocation in the primary
     * buffer.
     */
    inline size_t available_in_primary_buffer() const {
        Block* iterator = heap_start;

        size_t total_used = 0;

        while (iterator != nullptr &&
               get_region_of_block(iterator) != Region::SECONDARY) {

            if (iterator->used) {
                total_used += iterator->size + BLOCK_HEADER_SIZE;
            }

            iterator = iterator->next;
        }

        if (total_used == 0) {
            return primary_buffer_size;
        }

        if (total_used == primary_buffer_size) {
            return 0;
        }

        return primary_buffer_size - total_used - BLOCK_HEADER_SIZE;
    }

    /**
     * @return Number of bytes available for allocation in the secondary
     * buffer.
     */
    inline size_t available_in_secondary_buffer() const {

        Block* iterator = heap_top;

        size_t total_used = 0;

        while (iterator != nullptr &&
               get_region_of_block(iterator) != Region::PRIMARY) {

            if (iterator->used) {
                total_used += iterator->size + BLOCK_HEADER_SIZE;
            }

            iterator = iterator->previous;
        }

        if (total_used == 0) {
            return secondary_buffer_size;
        }

        if (total_used == secondary_buffer_size) {
            return 0;
        }

        return secondary_buffer_size - total_used - BLOCK_HEADER_SIZE;
    }

    /**
     * @brief Resets all the allocated memory.
     */
    inline void reset() {
        // Set up one empty block in the primary buffer and one empty block
        // in the secondary buffer. With that, we circumvent the case where
        // a buffer to big for the primary buffer gets allocated first and
        // then succeeding buffers are allocated in the primary buffer and
        // we need to reshuffle the block pointers.

        Block* secondary_block = (Block*)secondary_buffer;
        secondary_block->size  = secondary_buffer_size - BLOCK_HEADER_SIZE;
        secondary_block->used  = false;
        secondary_block->next  = nullptr;

        Block* primary_block    = (Block*)primary_buffer;
        primary_block->size     = primary_buffer_size - BLOCK_HEADER_SIZE;
        primary_block->used     = false;
        primary_block->next     = secondary_block;
        primary_block->previous = nullptr;

        secondary_block->previous = primary_block;

        heap_start = primary_block;
        heap_top   = secondary_block;
    }

#ifdef DEBUG
    /**
     * @brief Prints the blocks in the allocator.
     */
    inline void print() const {
        Block* iterator = heap_start;

        long count = 0;

        printf("================================= Allocator "
               "=================================\r\n\r\n");

        printf("%-20s 0x%-8lX -> 0x%-8lX, available: %8d/%d\r\n",
               "Primary buffer:",
               (long)primary_buffer,
               (long)primary_buffer + primary_buffer_size - 1,
               available_in_primary_buffer(),
               primary_buffer_size);

        printf("%-20s 0x%-8lX -> 0x%-8lX, available: %8d/%d\r\n",
               "Secondary buffer:",
               (long)secondary_buffer,
               (long)secondary_buffer + secondary_buffer_size - 1,
               available_in_secondary_buffer(),
               secondary_buffer_size);

        printf("\r\n");

        while (iterator != nullptr) {

            const Region region = get_region_of_block(iterator);

            long start;

            switch (region) {
            case Region::PRIMARY:
                start = (long)iterator - (long)primary_buffer;
                break;

            case Region::SECONDARY:
                start = (long)iterator - (long)secondary_buffer;
                break;
            }

            const long size = iterator->size + BLOCK_HEADER_SIZE;

            printf("%-8s  %-10s  %2ld: 0x%-8lX -> 0x%-8lX size: %8ld, in use: "
                   "%d\r\n",
                   iterator == heap_start
                       ? "Start"
                       : (iterator == heap_top ? "Top" : " "),
                   region == Region::PRIMARY ? "Primary" : "Secondary",
                   count++,
                   start,
                   start + size - 1,
                   size,
                   iterator->used);

            iterator = iterator->next;
        }
    }

    /**
     * @brief Checks that the doubly linked list is consistent. That is that the
     * next and previous pointers correspond.
     */
    inline bool check_consistency() {

        Block* block = heap_start;

        while (block != nullptr) {

            if (block->next) {
                const bool previous_same = block->next->previous == block;

                if (!previous_same) {
                    return false;
                }
            }

            if (block->previous) {
                const bool next_same = block->previous->next == block;

                if (!next_same) {
                    return false;
                }
            }

            block = block->next;
        }

        return true;
    }

#endif

  private:
    /**
     * @brief Used to differentiate the memory regions when doing merges and
     * splits.
     */
    enum class Region { PRIMARY = 0, SECONDARY = 1 };

    /**
     * @brief Pointer to the primary buffer, where the allocations will be
     * preferred to take place.
     */
    uint8_t* primary_buffer;

    /**
     * @brief Size of the primary buffer
     */
    size_t primary_buffer_size;

    /**
     * @brief Pointer to the secondary buffer, which will only be used when
     * the primary buffer is full or an allocation is too big for the
     * primary buffer.
     */
    uint8_t* secondary_buffer;

    /**
     * @brief Size of the secondary buffer.
     */
    const size_t secondary_buffer_size;

    /**
     * @brief Start of the heap for the allocator.
     */
    Block* heap_start = nullptr;

    /**
     * @brief Top of the heap.
     */
    Block* heap_top = heap_start;

    /**
     * @return The aligned size of @p size, returning upper 32-bit aligned
     * size.
     */
    static inline size_t align(const size_t size) {
        return (size + sizeof(uint32_t) - 1) & ~(sizeof(uint32_t) - 1);
    }

    /**
     * @brief Returns the bytes required to allocate a block with its
     * associated data.
     */
    static inline size_t allocation_size(const size_t data_size) {
        // Don't include the pointer in the block to the data here, as it is
        // included by the data size
        return data_size + BLOCK_HEADER_SIZE;
    }

    /**
     * @return The region of a @p block.
     */
    inline Region get_region_of_block(Block* block) const {

        // The secondary buffer can be placed at an address before the
        // primary buffer, so we have to do explicit checking for the
        // boundary
        const bool in_primary_buffer = (primary_buffer <= (uint8_t*)block) &&
                                       ((uint8_t*)block <
                                        primary_buffer + primary_buffer_size);

        return in_primary_buffer ? Region::PRIMARY : Region::SECONDARY;
    }

    /**
     * @return The block for the given @p data_pointer.
     */
    static inline Block* get_block(const void* data_pointer) {
        return (Block*)((uint8_t*)data_pointer - BLOCK_HEADER_SIZE);
    }

    /**
     * @return Whether a block can be split.
     */
    static inline bool can_split_block(Block* block, const size_t size) {
        // If the block minus the required header and the requested size at
        // least fits a new block (with 1 data element)
        return (long)(allocation_size(block->size) - size -
                      BLOCK_HEADER_SIZE) >= (long)sizeof(Block);
    }

    /**
     * @brief Splits the block, returning a new block adjusted for @p size.
     */
    inline Block* split(Block* block, const size_t size) {
        // Inject a free block in the middle
        Block* free_part = (Block*)((uint8_t*)block + allocation_size(size));
        free_part->size  = block->size - size - BLOCK_HEADER_SIZE;
        free_part->used  = false;
        free_part->next  = block->next;

        if (heap_top == block) {
            heap_top = free_part;
        }

        block->size = size;
        block->next = free_part;

        free_part->previous = block;

        // Update the block after the free part so that it points to the free
        // part
        if (free_part->next) {
            free_part->next->previous = free_part;
        }

        return block;
    }

    /**
     * @return Whether the current @p block and its previous and/or next can
     * be merged.
     */
    inline bool can_merge_block(Block* block) const {
        bool can_merge_next = false, can_merge_previous = false;

        const Region current_region = get_region_of_block(block);

        if (block->next) {
            const Region next_region = get_region_of_block(block->next);
            can_merge_next           = (current_region == next_region) &&
                             !block->next->used;
        }

        if (block->previous) {
            const Region previous_region = get_region_of_block(block->previous);
            can_merge_previous = (current_region == previous_region) &&
                                 !block->previous->used;
        }

        return can_merge_previous || can_merge_next;
    }

    /**
     * @brief Merges the current @p block with the previous and/or next one,
     * given they are in the same region.
     */
    inline Block* merge_block(Block* block) {

        Block* new_block = block;

        const Region current_region = get_region_of_block(block);

        if (block->previous) {

            const Region previous_region = get_region_of_block(block->previous);

            if (current_region == previous_region && !block->previous->used) {

                new_block = block->previous;
                new_block->size += block->size + BLOCK_HEADER_SIZE;
                new_block->next = block->next;

                if (new_block->next) {
                    new_block->next->previous = new_block;
                }

                if (block == heap_top) {
                    heap_top = new_block;
                }
            }
        }

        if (new_block->next) {

            const Region next_region = get_region_of_block(block->next);

            if (current_region == next_region && !new_block->next->used) {

                if (new_block->next == heap_top) {
                    heap_top = new_block;
                }

                new_block->size += new_block->next->size + BLOCK_HEADER_SIZE;
                new_block->next = new_block->next->next;

                if (new_block->next) {
                    new_block->next->previous = new_block;
                }
            }
        }

        return new_block;
    }

    /**
     * @brief O(n) first fit search for available block.
     */
    Block* first_fit(const size_t data_size) const {

        Block* block = heap_start;
        Block* best  = nullptr;

        while (block != nullptr) {
            if (!block->used && block->size >= data_size) {
                best = block;
                break;
            }

            block = block->next;
        }

        return best;
    }
};

#endif
