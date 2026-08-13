#include <stdio.h>
int main(void) {
    printf("bvr_crash_test_native: triggering AV\n");
    volatile int* p = (volatile int*)0;
    *p = 42;
    return 0;
}
