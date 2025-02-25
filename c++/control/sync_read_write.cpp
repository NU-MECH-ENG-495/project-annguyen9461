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

#include <string.h>
#include <chrono>
#include <thread>

#include <unordered_map>
#include <vector>
#include <sstream>
#include <iostream>


#define MAX_INPUT_SIZE 100      // define max input buffer size

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION           132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRESENT_POSITION            4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define NUM_MOTORS                      1                  // IDs: 17?


int DXL_ID;

void scan_motors(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler);

void set_torque(dynamixel::PacketHandler *packetHandler, 
                dynamixel::PortHandler *portHandler, 
                const char *command, char *ids_str); 

// Function to copy array
void copy_array(int* dest, int* src) {
    for (int i = 0; i <= NUM_MOTORS; i++) {
        dest[i] = src[i];
    }
}

int degree_to_pos_diff(int degree) {
  return static_cast<int>((degree/360.0) * 4095);   // used 360.0 to prevent zero for small angles
}

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

void scan_motors(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler)
{
  printf("Scanning for connected Dynamixel motors...\n");

  // Clear previous parameters
  groupSyncRead.clearParam();
  
  // Scan for active motors
  for (int id = 1; id <= NUM_MOTORS; id++) {
      int dxl_comm_result = packetHandler->ping(portHandler, id);
      if (dxl_comm_result == COMM_SUCCESS) {
          printf("Found Dynamixel ID: %d\n", id);

          // Add ID to GroupSyncRead
          bool dxl_addparam_result = groupSyncRead.addParam(id);
          if (!dxl_addparam_result) {
              printf("Failed to add ID %d to SyncRead\n", id);
          }
      }
  }

  // Read all present positions
  int dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
      printf("Failed to read positions: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Print present positions of connected motors
  printf("\nCurrent Positions:\n");
  for (int id = 1; id <= NUM_MOTORS; id++) {
      if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
          int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          printf("[ID:%d] Position: %d\n", id, position);
      }
  }
  printf("\n");
}

void print_present(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler)
{
    // Read all present positions
    int dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("Failed to read positions: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Print present positions of connected motors
    printf("\nCurrent Positions:\n");
    for (int id = 1; id <= NUM_MOTORS; id++) {
        if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
            int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            printf("[ID:%d] Position: %d\n", id, position);
        }
    }
    printf("\n");
}

void update_present_positions(dynamixel::GroupSyncRead &groupSyncRead, 
                 dynamixel::PacketHandler *packetHandler, 
                 dynamixel::PortHandler *portHandler)
{
  // Clear previous parameters
  groupSyncRead.clearParam();

  // Scan for active motors
  for (int id = 1; id <= NUM_MOTORS; id++) {
      int dxl_comm_result = packetHandler->ping(portHandler, id);
      if (dxl_comm_result == COMM_SUCCESS) {
          // Add ID to GroupSyncRead
          bool dxl_addparam_result = groupSyncRead.addParam(id);
      }
  }

  // Read all present positions
  int dxl_comm_result = groupSyncRead.txRxPacket();

  // Update present positions of connected motors
  for (int id = 1; id <= NUM_MOTORS; id++) {
      if (groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
          int32_t position = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          present_positions[id] = position;
      }
  }
  // printf("Present motor positions UPDATED\n");
  // for (int id = 1; id <= NUM_MOTORS; id++) {
  //   printf("[ID: %d] Position: %d\n", id, present_positions[id]);
  // }
}

void update_one_motor_pos(dynamixel::GroupSyncRead &groupSyncRead, 
  dynamixel::PacketHandler *packetHandler, 
  dynamixel::PortHandler *portHandler, 
  int motor_id) // Only update this motor
{
  // Clear previous parameters
  groupSyncRead.clearParam();

  // Add only the specified motor for reading
  bool dxl_addparam_result = groupSyncRead.addParam(motor_id);
  if (!dxl_addparam_result) {
  printf("Failed to add ID %d to SyncRead\n", motor_id);
  return;
  }

  // Read position for the selected motor
  int dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
  printf("Failed to read position for ID %d: %s\n", motor_id, packetHandler->getTxRxResult(dxl_comm_result));
  return;
  }

  // Update only the specified motor
  if (groupSyncRead.isAvailable(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
  int32_t position = groupSyncRead.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
  present_positions[motor_id] = position;
  printf("[ID:%d] Updated Position: %d\n", motor_id, position);
  }
}

void set_torque(dynamixel::PacketHandler *packetHandler, 
                dynamixel::PortHandler *portHandler, 
                const char *command, char *ids_str)
{
  bool enable = (strcmp(command, "en") == 0) ? TORQUE_ENABLE : TORQUE_DISABLE;
  uint8_t dxl_error = 0;

  char *token = strtok(ids_str, " ");
  while (token != NULL) {
      int dxl_id = atoi(token);
      if (dxl_id < 1 || dxl_id > NUM_MOTORS) {
          printf("Invalid ID: %d. Must be between 1 and %d.\n", dxl_id, NUM_MOTORS);
      } else {
          int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, enable, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS) {
              printf("[ID:%d] Torque change failed: %s\n", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
          } else {
              printf("[ID:%d] Torque %s\n", dxl_id, enable ? "ENABLED" : "DISABLED");
          }
      }
      token = strtok(NULL, " ");
  }
}

void move_to_target_positions(
                      int* target_positions,
                      dynamixel::GroupSyncWrite &groupSyncWrite, 
                      dynamixel::PacketHandler *packetHandler 
                      )
{
  printf("Moving motors to %s...\n", toggle_position ? "TOP RIGHT up" : "TOP LEFT up");

  // Clear previous SyncWrite parameters
  groupSyncWrite.clearParam();

  uint8_t param_goal_position[4];
  int goal_position = target_positions[DXL_ID];

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

  // Add goal position to SyncWrite buffer
  if (!groupSyncWrite.addParam(DXL_ID, param_goal_position)) {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addParam failed\n", DXL_ID);
      continue;
  }

  // Transmit the target positions to all motors at once
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Clear SyncWrite buffer after sending data
  groupSyncWrite.clearParam();
}


void gradual_transition(int* next_positions, 
                        dynamixel::GroupSyncWrite &groupSyncWrite, 
                        dynamixel::PacketHandler *packetHandler) {
    const int step_size = 17;  // number of steps for smooth transition
    int step_arr[NUM_MOTORS + 1] = {0};
    int num_motors = NUM_MOTORS;
    // loop range to start from 1 (ignoring index 0)

    int updated_positions[NUM_MOTORS + 1];
    std::copy(std::begin(present_positions), std::end(present_positions), std::begin(updated_positions));

    for (int i = 1; i <= num_motors; i++) {
        step_arr[i] = (next_positions[i] - updated_positions[i]) / step_size;
    }
    // perform transitions for each step
    for (int step = 0; step < step_size; step++) {
        // update motor transition based on respective step size
        for (int i = 1; i <= num_motors; i++) {
            updated_positions[i] += step_arr[i];
        }
        move_to_target_positions(updated_positions, groupSyncWrite, packetHandler);
    }
    // ensure final position is accurate (due to integer division)
    move_to_target_positions(next_positions, groupSyncWrite, packetHandler);
}

int main() 
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result

  uint8_t dxl_error = 0;                            // Dynamixel error

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


  for (int i = 0; i < 20; i++) {
    DXL_ID = i;
    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
      printf("Dynamixel#%d has been successfully connected \n", DXL_ID);
    }

    // Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID);
      return 0;
    }
  }


  std::string input;

  while (true) {
    // Get user input
    std::cout << "Enter command: ";
    std::getline(std::cin, input);

    // Trim leading/trailing spaces
    if (input.empty()) continue;

    // Handle exit condition
    if (input == "exit") break;

    // Extract first word as command
    std::istringstream iss(input);
    std::string command;
    iss >> command;

    if (command == "get") {
      scan_motors(groupSyncRead, packetHandler, portHandler);
    }

    // else if (command == "h1") {
    //   move_to(home_tiptoe, groupSyncWrite, packetHandler,groupSyncRead, portHandler);
    // }
    
    else {
      std::cout << "Unknown command: " << command << "\n";
    }
  }

  // for (int i = 0; i < 20; i++) {
  //   // Disable Dynamixel# Torque
  //   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  //   if (dxl_comm_result != COMM_SUCCESS)
  //   {
  //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  //   }
  //   else if (dxl_error != 0)
  //   {
  //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  //   }
  // }

// Close port
portHandler->closePort();

return 0;

}
