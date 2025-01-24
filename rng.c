#include <stdio.h>
#include <stdlib.h>

int get_rand(int *seed) {
    *seed = (*seed * 5678 + 1234) % 1928;
    if (abs(*seed) > 10000)
    {
        *seed = (int)(*seed / 10000);
        // printf("Number exceeds the criteria.\n");
    }
    if (abs(*seed) % 2 == 0)
    {
        *seed = -*seed;
        // printf("Number is negative.\n");
    }

    return *seed;
}

int main() {
    int seed = 16;
    int number;

    while (1) { 
        number = get_rand(&seed);
        printf("Generated number: %d\n", number);
        if (abs(number) < 100) {
            break;
        }
    }

    printf("Number %d meets the criteria.\n", number);

    return 0;
}
