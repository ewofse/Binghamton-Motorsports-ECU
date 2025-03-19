// Safe guards
#ifndef BUFFER_H
#define BUFFER_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"

/*-------------------------------------------------------------------------------------------------
 Circular Buffer with a Total Sum
-------------------------------------------------------------------------------------------------*/
class circularBuffer {
    public:
        // Constructor
        circularBuffer(const size_t elements);

        // Getters
        size_t GetCapacity() { return capacity; }
        size_t GetCount() { return count; }
        uint32_t GetTotal() { return total; }

        // Data methods
        void FreeBuffer();
        void PushBuffer(uint16_t value);
        uint16_t PullBuffer();

    private:
        // Data buffer and pointers to current start and end of buffer
        uint16_t * pBuffer;
        uint16_t * pHead;
        uint16_t * pTail;
        uint32_t total;

        // Number of elements inside the buffer and maximum number of elements
        size_t count;
        size_t capacity;
};

// End safe guards
#endif /* BUFFER_H */
