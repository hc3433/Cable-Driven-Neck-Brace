#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <WinSock2.h>

#pragma comment (lib,"ws2_32.lib")

#define BUFLEN 512
#define PORT 12354

void error(const char* msg);
double get_time();