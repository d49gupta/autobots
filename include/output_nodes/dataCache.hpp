#include <vector>

class dataCache {
public:
    dataCache(int size) : size(size), buffer(size), head(0), tail(0), count(0) {}
    bool isFull();
    bool isEmpty();
    void enqueue(int data);
    int newestValue();

private:
    int size;
    std::vector<int> buffer = {}; 
    int head;
    int tail;
    int count;
};