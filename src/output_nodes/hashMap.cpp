#include "hashMap.hpp"
#include <string>
#include <iostream>

int main() {
    HashMap<Time, int> hashMap(3);
    Time t;
    t.seconds = 1403636599;
    t.nanoseconds = 753555500;

    Time t1;
    t1.seconds = 1403636598;
    t1.nanoseconds = 963555500;

    Time t2;
    t2.seconds = 1403636598;
    t2.nanoseconds = 963555500;

    hashMap.add(t, 10);
    hashMap.add(t1, 20);
    hashMap.add(t2, 50);
    // hashMap.add(1403636599773555500, 30);
    // hashMap.add(1403636599783555500, 40);

    // hashMap.add(1403636599853555500, 100);
    // hashMap.add(1403636599953555500, 200);

    // std::cout << "Newest value for sensor1: " << hashMap.getNewest(1403636599753555500) << std::endl;
    // std::cout << "Newest value for sensor2: " << hashMap.getNewest(1403636599753555500) << std::endl;

    hashMap.print();

    return 0;
}