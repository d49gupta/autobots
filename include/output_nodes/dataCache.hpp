#include <vector>

class dataCache {
public:
    dataCache(int size) : size(size), buffer(size), head(0), tail(0), count(0) {}
    bool isFull();
    bool isEmpty();
    void enqueue(double data);
    double newestValue();

private:
    std::vector<double> buffer = {};
    int head;
    int tail;
    int size;
    int count;
}