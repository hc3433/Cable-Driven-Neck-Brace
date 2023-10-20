#include "SaveData.h"
#include <time.h>
FILE* createFile(int nMotors) {
#include <time.h>

    // Get the current system time
    time_t rawtime;
    struct tm* timeinfo;
    char filename[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    // Format the time into "YYYYMMDD_HHMMSS.csv" format
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S.csv", timeinfo);

    // Create a CSV file with the system time as its name
    FILE* dataFile = fopen(filename, "w");
    if (!dataFile) {
        perror("Failed to open data file");
        return 0;
    }

    // Write headers to CSV
    fprintf(dataFile, "Time,dxl1_position,dxl2_position,sol_x0,sol_x1,sol_x2,sol_x3,sol_x4,sol_x5,sol_x6\n");
    
    fprintf(dataFile, "Time"); // Time Headers
    // Motion Position
    fprintf(dataFile)
    return dataFile;

};