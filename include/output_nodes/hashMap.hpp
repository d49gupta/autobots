#ifndef HASHMAP_HPP
#define HASHMAP_HPP

#include "dataCache.hpp"
#include <vector>
#include <cstddef>
#include <iostream>

template <typename K, typename T>
class Node {
public:
    K key;               
    dataCache<T> cache;  

    Node(const K& key, int cacheSize) : key(key), cache(cacheSize) {}
};

template <typename K, typename T>
class HashMap {
private:
    std::vector<dataCache<T>> table;
    size_t bucketCount;

    size_t hash(const K& key) const {
        return std::hash<K>()(key) % bucketCount;
    }

public:
    HashMap(size_t buckets, int cacheSize) : bucketCount(buckets) {
        table.resize(bucketCount, dataCache<T>(cacheSize));
    }

    void add(const K& key, const T& value) {
        size_t bucketIndex = hash(key);
        table[bucketIndex].enqueue(value);
    }

    T getNewest(const K& key) const {
        size_t bucketIndex = hash(key);
        return table[bucketIndex].newestValue();
    }

    void print() const {
        for (size_t i = 0; i < bucketCount; ++i) {
            std::cout << "Bucket " << i << ": ";
            if (!table[i].isEmpty()) {
                std::cout << "Newest Value: " << table[i].newestValue();
            } else {
                std::cout << "Empty";
            }
            std::cout << std::endl;
        }
    }
};
#endif