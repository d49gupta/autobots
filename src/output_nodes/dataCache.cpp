#include "dataCache.hpp"
#include <iostream>

bool dataCache::isFull() {
    return this->count == this->size;
}

bool dataCache::isEmpty() {
    return this->count == 0;
}

void dataCache::enqueue(double data) {
    buffer[head] = data;
    head = (head + 1) % size;

    if (isFull()) {
        tail = (tail + 1) % size;
    }
    else {
        count += 1;
    }
}

double dataCache::newestValue() {
    if (!isEmpty()) {
        int newestIndex = (head - 1 + size) % size;
        return buffer[newestIndex];
    }
    else {
        return -1;
    }
}

int main() {
	dataCache circularBuffer(3);
    circularBuffer.enqueue(10);
    circularBuffer.enqueue(9);
    std::cout<<circularBuffer.newestValue()<<std::endl;
    circularBuffer.enqueue(8);
    std::cout<<circularBuffer.newestValue()<<std::endl;

    return 0;
}