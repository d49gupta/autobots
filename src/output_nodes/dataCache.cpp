#include "dataCache.hpp"

bool dataCache::isFull() {
    return this->count == this->size;
}

bool dataCache::isEmpty() {
    return this->count == 0;
}

void dataCache::enqueue(double data) {
    buffer[head] = data;
    head = (head + 1) % size;

    if isFull() {
        tail = (tail + 1) % size;
    }
    else {
        count += 1;
    }
}

dobule dataCache::newestValue() {
    if !isEmpty() {
        newestIndex = (head - 1) % size;
        return buffer[newestIndex];
    }
    else {
        return nullptr;
    }
}