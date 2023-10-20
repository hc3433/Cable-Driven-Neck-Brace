#include "UDPechoServer.h"

// A function to print an error message and exit
void error(const char* msg) {
    perror(msg);
    exit(1);
}

// A function to get the current time in seconds
double get_time() {
    LARGE_INTEGER t, f;
    QueryPerformanceCounter(&t);
    QueryPerformanceFrequency(&f);
    return (double)t.QuadPart / (double)f.QuadPart;
}
