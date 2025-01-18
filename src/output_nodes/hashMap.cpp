#include "hashMap.hpp"
#include <string>
#include <iostream>

int main() {
    // Create a hash map with 5 buckets, each bucket can store up to 3 values
    HashMap<std::string, int> hashMap(5, 3);

    // Add values to the hash map
    hashMap.add("sensor1", 10);
    hashMap.add("sensor1", 20);
    hashMap.add("sensor1", 30);
    hashMap.add("sensor1", 40); // This will overwrite the oldest value in the circular queue

    hashMap.add("sensor2", 100);
    hashMap.add("sensor2", 200);

    // Retrieve and print the newest value for a key
    std::cout << "Newest value for sensor1: " << hashMap.getNewest("sensor1") << std::endl;
    std::cout << "Newest value for sensor2: " << hashMap.getNewest("sensor2") << std::endl;

    // Print the entire hash map
    hashMap.print();

    return 0;
}