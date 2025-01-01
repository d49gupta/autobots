#ifndef DATACACHE_HPP
#define DATACACHE_HPP

#include <vector>
template <typename T>
class dataCache {
public:
    dataCache(int size) : size(size), buffer(size), head(0), tail(0), count(0) {}
    bool isFull() {
        return this->count == this->size;
    }

    bool isEmpty() {
        return this->count == 0;
    }

    void enqueue(const T& data) {
        buffer[head] = data;
        head = (head + 1) % size;

        if (isFull()) {
            tail = (tail + 1) % size;
        }
        else {
            count += 1;
        }
    }

    T newestValue() {
        if (!isEmpty()) {
            int newestIndex = (head - 1 + size) % size;
            return buffer[newestIndex];
        }
        else {
            return T{};
        }
    }

private:
    int size;
    std::vector<T> buffer = {}; 
    int head;
    int tail;
    int count;
};

#endif