#include "sensors/buffer.h"

/*-----------------------------------------------------------------------------
 Circular buffer construtor
-----------------------------------------------------------------------------*/
circularBuffer::circularBuffer(const size_t elements) {
    // Check there are a non-zero number of elements
    if (elements > 0) {
        // Declare memory for buffer and set head and tail start and end
        pBuffer = static_cast<uint16_t *>( calloc( elements, sizeof(uint16_t) ) );
        pHead = pBuffer + elements - 1;
        pTail = pBuffer;

        // Initialize sum of buffer & number of elements
        total = 0;
        count = 0;
        capacity = elements;
    } else {
        // Set all pointes to NULL
        pBuffer = NULL;
        pHead = NULL;
        pTail = NULL;

        // Set all counters to zero
        total = 0;
        count = 0;
        capacity = 0;
    }
}

/*-----------------------------------------------------------------------------
 Add element to circular buffer
-----------------------------------------------------------------------------*/
void circularBuffer::PushBuffer(uint16_t value) {
    // Check the buffer and capacity were initizlied
    if (pBuffer && capacity > 0) {
        // Subtract old value from total sum
        total -= *pHead;

        // Store inputted data into current buffer head
        *pHead = value;

        // Add new value to total sum
        total += *pHead;

        // Wrap around when reaching the end of buffer
        if (pHead == pBuffer + capacity) {
            pHead = pBuffer;
        }

        // Advance the head to the next element
        ++pHead;

        // Update current element counter and tail
        if (count == capacity) {
            // Advance the tail to the next element
            ++pTail;

            // Wrap around when reaching the end of buffer
            if (pTail == pBuffer + capacity) {
                pTail = pBuffer;
            }
        } else {
            // Increment the element count
            ++count;
        }
    }
}

/*-----------------------------------------------------------------------------
 Obtain an element from circular buffer
-----------------------------------------------------------------------------*/
uint16_t circularBuffer::PullBuffer() {
    uint16_t value = 0;

    // Check the buffer was initialized
    if (pBuffer) {
        // Obtain the value at the head of buffer if the head is valid
        value = (pHead >= pBuffer) && (pHead < pBuffer + count - 1) ? *pHead : 0;
    }
    
    return value;
}

/*-----------------------------------------------------------------------------
 Obtain the average value inside the buffer
-----------------------------------------------------------------------------*/
uint16_t circularBuffer::GetAverage() {
    // Check for a non-zero element count
    return (count > 0) ? total / count : 0;
}

/*-----------------------------------------------------------------------------
 Free all allocated memory from object
-----------------------------------------------------------------------------*/
void circularBuffer::FreeBuffer() {
    // Check the buffer is not NULL
    if (pBuffer) {
        // Free memory (this includes head & tail)
        free(pBuffer);

        // Set pointer to NULL to prevent dangling pointer
        pBuffer = NULL;
    }
}
