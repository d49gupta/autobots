#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

using namespace std;

class csvReader 
{
public:
    csvReader(string filePath) : filePath(filePath) {}
    vector<string> split(const string &str, char delimiter);
    void readCSV();
    void printRow(map<string, string> row);
    vector<map<string, string>> data;

private:
    string filePath;
};
