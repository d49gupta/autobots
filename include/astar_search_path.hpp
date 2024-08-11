#include <bits/stdc++.h>
using namespace std;

extern int ROW;
extern int COL;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int> > pPair;

// A structure to hold the necessary parameters
struct cell {
	// Row and Column index of its parent
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	int parent_i, parent_j;
	// f = g + h
	double f, g, h;
};

bool isValid(int row, int col);
bool isUnBlocked(std::vector<std::vector<int>> grid, int row, int col);
bool isDestination(int row, int col, Pair dest);
double calculateHValue(int row, int col, Pair dest);
void tracePath(std::vector<std::vector<cell>> cellDetails, Pair dest);
void aStarSearch(std::vector<std::vector<int>> grid, Pair src, Pair dest);
void useGlobalVariables();