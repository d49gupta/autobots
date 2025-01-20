#ifndef HASHMAP_HPP
#define HASHMAP_HPP

#include "dataCache.hpp"
#include <map>
#include <cstddef>
#include <iostream>
#include <cmath>

#define GPS_FREQUENCY 100000000 //in nanoseconds
#define OFFSET 1000000000000000000

struct Time {
    long int seconds;
    long int nanoseconds;
};

template <typename K, typename T>
class HashMap {
private:
    std::map<size_t, dataCache<T>> table;
    double bucket_interval = 1.0 / GPS_FREQUENCY;
    int cacheSize;

    size_t hash(const K& key) const {
        //time = key.seconds*10e9 + key.nanoseconds
        // double normalized_time = static_cast<size_t>((key - OFFSET) * bucket_interval);
        // double seconds;
        // double nanoseconds = std::modf(normalized_time, &seconds);
        int left_two_digits = static_cast<int>(key.seconds) % 100;
        int right_two_digits = getFirstTwoDigits(key.nanoseconds);
        // std::cout<<left_two_digits<<" "<<right_two_digits<<std::endl;
        size_t index = left_two_digits*100 + right_two_digits;
        // std::cout << "This is the index: " << index << std::endl;
        return index; 
    }

public:
    HashMap(int cacheSize): cacheSize(cacheSize) {}

    int getFirstTwoDigits(long long number) const {
        int numDigits = static_cast<int>(std::log10(number)) + 1;
        long long divisor = static_cast<long long>(std::pow(10, numDigits - 2));
        return number / divisor;
    }

    void add(const K& key, const T& value) {
        size_t bucketIndex = hash(key);
        if (table.find(bucketIndex) != table.end()) 
            table[bucketIndex].enqueue(value);
        else
        {
            dataCache<T> newCache(cacheSize);
            newCache.enqueue(value);
            table[bucketIndex] = newCache;
        }
    }

    T getNewest(const K& key) const {
        size_t bucketIndex = hash(key);
        if (table.find(bucketIndex) == table.end())
        {
            //add logic to interpolate nearest imu value
            throw std::runtime_error("Bucket not found");
        }
        return table.at(bucketIndex).newestValue();
    }

    void print() const {
       for (const auto& pair : table) { 
            size_t bucketIndex = pair.first;
            const auto& queue = pair.second;
            std::cout << "Bucket " << bucketIndex << ": ";
            if (!queue.isEmpty()) {
                std::cout << "Newest Value: " << queue.newestValue();
            } 
            else {
                std::cout << "Empty";
            }
            std::cout << std::endl;
        }
    }
};
#endif