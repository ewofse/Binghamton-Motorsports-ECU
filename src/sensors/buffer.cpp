#include "sensors/buffer.h"

/*-----------------------------------------------------------------------------
 Circular buffer construtor
-----------------------------------------------------------------------------*/
circularBuffer::circularBuffer(const size_t elements) {
    // Declare memory for buffer and set head and tail start and end
    pBuffer = (uint16_t *) calloc( elements, sizeof(uint16_t) );
    pHead = pBuffer + elements - 1;
    pTail = pBuffer;

    // Initialize sum of buffer & number of elements
    total = 0;
    count = 0;
    capacity = elements;
}

/*-----------------------------------------------------------------------------
 Add element to circular buffer
-----------------------------------------------------------------------------*/
void circularBuffer::PushBuffer(uint16_t value) {
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
        ++count;
    }
}

/*-----------------------------------------------------------------------------
 Obtain an element from circular buffer
-----------------------------------------------------------------------------*/
uint16_t circularBuffer::PullBuffer() {
    // Return the value at the head of buffer if the head is valid
    return (pHead >= pBuffer) && (pHead < pBuffer + count - 1) ? *pHead : 0;
}

/*-----------------------------------------------------------------------------
 Free all allocated memory from object
-----------------------------------------------------------------------------*/
void circularBuffer::FreeBuffer() {
    // Free memory (this includes head & tail)
    free(pBuffer);

    // Set pointer to null to prevent dangling pointer
    pBuffer = NULL;
}
