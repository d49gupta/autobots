#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

using namespace std;

// Function to split a string by a delimiter (comma)
vector<string> split(const string &str, char delimiter) {
    vector<string> result;
    stringstream ss(str);
    string token;
    while (getline(ss, token, delimiter)) {
        result.push_back(token);
    }
    return result;
}

// Function to read CSV and store the data
vector<map<string, string>> readCSV(const string &filename) {
    ifstream file(filename);
    vector<map<string, string>> csv_vector;

    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return csv_vector;
    }

    string line;
    bool first_row = true;
    vector<string> headers;

    // Read the file line by line
    while (getline(file, line)) {
        vector<string> row = split(line, ',');

        if (first_row) {
            headers = row;
            first_row = false;
        } 
        else {
            map<string, string> data_row; 
            for (size_t i = 0; i < row.size(); ++i) {
                data_row[headers[i]] = row[i];
            }
            csv_vector.push_back(data_row);
        }
    }

    file.close();
    return csv_vector;
}

void printRow(map<string, string> row) {
    for (const auto &pair : row) {
        cout << pair.first << ": " << pair.second << " | ";
    }
}

int main() {
    string filename = "../rosbags/data/imu0.csv";

    vector<map<string, string>> data = readCSV(filename);
    printRow(data[0]);
    printRow(data[4]);
    printRow(data[5]);

    return 0;
}
