#include "dataCache.hpp"
#include <iostream>

bool dataCache::isFull() {
    return this->count == this->size;
}

bool dataCache::isEmpty() {
    return this->count == 0;
}

void dataCache::enqueue(int data) {
    buffer[head] = data;
    head = (head + 1) % size;

    if (isFull()) {
        tail = (tail + 1) % size;
    }
    else {
        count += 1;
    }
}

int dataCache::newestValue() {
    if (!isEmpty()) {
        int newestIndex = (head - 1 + size) % size;
        return buffer[newestIndex];
    }
    else {
        return -1;
    }
}