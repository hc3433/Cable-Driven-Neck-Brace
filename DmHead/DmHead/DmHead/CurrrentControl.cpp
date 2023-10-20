#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_CURRENT           102
#define ADDR_PRO_PRESENT_CURRENT        126
#define ADDR_OPERATING_MODE             11


// Control Mode
#define EXT_CURRENT_MODE                1

// Data Byte Length
#define LEN_PRO_GOAL_CURRENT             4
#define LEN_PRO_PRESENT_CURRENT          4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         4                   // Dynamixel#1 ID: 1
//#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
//#define DXL3_ID                         3                   // Dynamixel#3 ID: 3
#define BAUDRATE                        57600
#define DEVICENAME                      "COM3"              // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_CURRENT_VALUE       300                  // Dynamixel will rotate between this value
#define DXL_MAXIMUM_CURRENT_VALUE       500               // and this value (note that the Dynamixel would not move when the current value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

int main()
{
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT);

    // Initialize Groupsyncread instance for Present Current
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result
    int dxl_goal_current[2] = { DXL_MINIMUM_CURRENT_VALUE, DXL_MAXIMUM_CURRENT_VALUE };  // Goal current

    uint8_t dxl_error = 0;                            // Dynamixel error
    uint8_t param_goal_current[4];
    int32_t dxl1_present_current = 0, dxl2_present_current = 0;                         // Present current

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set operating mode to extended force control mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING_MODE, EXT_CURRENT_MODE, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Operating mode changed to extended current control mode. \n");
    }
    /*dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERATING_MODE, EXT_CURRENT_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Operating mode changed to extended current control mode. \n");
    }
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_OPERATING_MODE, EXT_CURRENT_MODE, &dxl_error);
    */
    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    /* Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
      printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
    */
    /* Enable Dynamixel#3 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
    }
    */
    // Add parameter storage for Dynamixel#1 present current value
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
        return 0;
    }

    /* Add parameter storage for Dynamixel#2 present current value
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
      return 0;
    }
    */
    /* Add parameter storage for Dynamixel#3 present current value
    dxl_addparam_result = groupSyncRead.addParam(DXL3_ID);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL3_ID);
        return 0;
    }
    */
    while (1)
    {
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;

        // Allocate goal current value into byte array
        param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_current[index]));
        param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_current[index]));
        param_goal_current[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_current[index]));
        param_goal_current[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_current[index]));

        // Add Dynamixel#1 goal current value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_current);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
            return 0;
        }

        /* Add Dynamixel#2 goal current value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_current);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
          return 0;
        }
        */
        /* Add Dynamixel#3 goal current value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_current);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
            return 0;
        }
        */
        // Syncwrite goal current
        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

        // Clear syncwrite parameter storage
        groupSyncWrite.clearParam();

        do
        {
            // Syncread present current
            dxl_comm_result = groupSyncRead.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (groupSyncRead.getError(DXL1_ID, &dxl_error))
            {
                printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
            }
            /* else if (groupSyncRead.getError(DXL2_ID, &dxl_error))
             {
               printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
             }
             else if (groupSyncRead.getError(DXL3_ID, &dxl_error))
             {
                 printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
             }*/
             // Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);
            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
                return 0;
            }

            /* Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);
            if (dxl_getdata_result != true)
            {
              fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
              return 0;
            }
            */
            /* Check if groupsyncread data of Dynamixel#3 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);
            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL3_ID);
                return 0;
            }
            */
            // Get Dynamixel#1 present current value
            dxl1_present_current = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);

            // Get Dynamixel#2 present current value
            //dxl2_present_current = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);

            // Get Dynamixel#3 present current value
            //dxl2_present_current = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);

            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_current[index], dxl1_present_current);

        } while ((abs(dxl_goal_current[index] - dxl1_present_current) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_current[index] - dxl2_present_current) > DXL_MOVING_STATUS_THRESHOLD));

        // Change goal current
        if (index == 0)
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }

    // Disable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    /* Disable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    */
    /* Disable Dynamixel#3 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    */
    // Close port
    portHandler->closePort();

    return 0;
}
