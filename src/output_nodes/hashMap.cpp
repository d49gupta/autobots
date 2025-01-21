#include "hashMap.hpp"
#include <string>
#include <iostream>

int main() {
    HashMap<Time, int> hashMap(3, 3);
    Time t;
    t.seconds = 1403636599;
    t.nanoseconds = 753555500;

    Time t1;
    t1.seconds = 1403636598;
    t1.nanoseconds = 963555500;

    Time t2;
    t2.seconds = 1403636598;
    t2.nanoseconds = 964655500;

    Time t3;
    t3.seconds = 1403636598;
    t3.nanoseconds = 962655500;

    Time t4;
    t4.seconds = 1403636598;
    t4.nanoseconds = 967655500;

    hashMap.add(t, 10);
    hashMap.add(t1, 20);
    hashMap.add(t2, 50);
    hashMap.print();

    int newestValue = hashMap.getNewest(t3);
    std::cout<<"Newest Value is: "<<newestValue<<std::endl;

    int thing = hashMap.getNewest(t4);
    std::cout<<"Newest Value is: "<<thing<<std::endl;

    return 0;
}