#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used

// Add you code below



// Allocation bitmap: 0 = free, 1 = used
static uint8_t block_map[BLOCK_COUNT];  // 0 = free, 1 = used

// -----------------------------------
// Initialize heap: mark all blocks free
// -----------------------------------
void heap_init(void) {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        block_map[i] = 0;
    }
}

// -----------------------------------
// Allocate memory from heap
// size: number of bytes requested
// Returns pointer to memory or NULL if not enough space
// -----------------------------------
void* heap_alloc(size_t size) {
    if (size == 0 || size > HEAP_SIZE) return NULL;

    int blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;  // ceil division
    int free_count = 0;

    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (block_map[i] == 0) {
            free_count++;
        } else {
            free_count = 0;
        }

        if (free_count == blocks_needed) {
            // Mark blocks as used
            for (int j = i - blocks_needed + 1; j <= i; j++) {
                block_map[j] = 1;
            }

            return HEAP_START_ADDR + (i - blocks_needed + 1) * BLOCK_SIZE;
        }
    }

    return NULL; // Not enough contiguous space
}

// -----------------------------------
// Free previously allocated memory
// ptr: pointer returned by heap_alloc
// -----------------------------------
void heap_free(void* ptr) {
    if (!ptr) return;

    uintptr_t offset = (uint8_t*)ptr - HEAP_START_ADDR;
    if (offset >= HEAP_SIZE) return; // Pointer outside heap

    int start_block = offset / BLOCK_SIZE;

    // Free consecutive used blocks starting from start_block
    for (int i = start_block; i < BLOCK_COUNT && block_map[i] == 1; i++) {
        block_map[i] = 0;
    }
}