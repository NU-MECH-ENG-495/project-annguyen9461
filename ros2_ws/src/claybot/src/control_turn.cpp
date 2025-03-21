#include "control_turn.hpp"

/**
 * @brief Control table address for X series (except XL-320)
 * These addresses are specific to the Dynamixel X series protocol
 */
#define ADDR_OPERATING_MODE 11     /**< Address for setting the operating mode of the motor */
#define ADDR_TORQUE_ENABLE 64      /**< Address to enable/disable motor torque */
#define ADDR_GOAL_POSITION 116     /**< Address to set the target position */
#define ADDR_PRESENT_POSITION 132  /**< Address to read the current position */

/**
 * @brief Protocol version
 */
#define PROTOCOL_VERSION 2.0  /**< Default Protocol version of DYNAMIXEL X series */

/**
 * @brief Default settings
 */
#define BAUDRATE 57600           /**< Default Baudrate of DYNAMIXEL X series */
#define DEVICE_NAME "/dev/ttyUSB0"  /**< Device name: [Linux]: "/dev/ttyUSB*", [Windows]: "COM*" */
#define NUM_MOTORS 1             /**< Number of motors connected to the system */

/**
 * @brief Global variables for Dynamixel communication
 */
uint8_t dxl_error = 0;          /**< Error status from Dynamixel communication */
uint32_t goal_position = 0;     /**< Target position for motor */
int dxl_comm_result = COMM_TX_FAIL;  /**< Communication result status */


/**
 * @brief Constructor for the ControlTurn class
 * 
 * Initializes ROS node, sets up parameters, subscribers, and services
 * 
 * To test manual turn:
 * ros2 topic pub --once /set_position claybot_interfaces/msg/SetPosition "{id: 1, position: 512}"
 */
ControlTurn::ControlTurn() : Node("control_turn_node")
{
    // Log node initialization
    RCLCPP_INFO(this->get_logger(), "Run control_turn_node node");

    // Declare and get QoS depth parameter
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    // Set up Quality of Service profile for reliable communication
    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    // Initialize Dynamixel port and packet handlers
    this->portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize group sync write instance for synchronously controlling multiple motors
    this->groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

    // Set up Dynamixel motors
    this->initDynamixels();

    /**
     * @brief Subscription handler for position commands
     * 
     * Create subscription to listen for position commands and implement smooth movement
     */
    set_position_subscriber_ =
        this->create_subscription<SetPosition>(
        "set_position",
        QOS_RKL10V,
        [this](const SetPosition::SharedPtr msg) -> void
        {
            // Implement smooth movement when new position is requested
            gradual_transition((int)msg->position);
        }
        );

    /**
     * @brief Service callback for getting motor position
     * 
     * Reads and returns the current position of the specified motor
     * 
     * @param request Contains the motor ID
     * @param response Contains the current position after execution
     */
    auto get_present_position =
        [this](
        const std::shared_ptr<GetPosition::Request> request,
        std::shared_ptr<GetPosition::Response> response) -> void
        {
        // Read Present Position (length: 4 bytes) and Convert uint32 -> int32
        // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            (uint8_t) request->id,
            ADDR_PRESENT_POSITION,
            reinterpret_cast<uint32_t *>(&present_position),
            &dxl_error
        );

        // Log the position information
        RCLCPP_INFO(
            this->get_logger(),
            "Get [ID: %d] [Present Position: %d]",
            request->id,
            present_position
        );

        // Set the response with the current position
        response->position = present_position;
        };

    // Register the service with ROS
    get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

/**
 * @brief Destructor for the ControlTurn class
 * 
 * Currently empty, but could handle cleanup tasks
 */
ControlTurn::~ControlTurn()
{
    // No cleanup tasks defined, but could be extended to:
    // - Disable torque
    // - Close port
    // - Delete dynamically allocated objects
}

/**
 * @brief Initialize the Dynamixel motors
 * 
 * Opens serial port, sets baudrate, configures operating mode, and enables torque
 */
void ControlTurn::initDynamixels()
{
    // Open Serial Port
    dxl_comm_result = this->portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("control_turn_node"), "Failed to open the port!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("control_turn_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("control_turn_node"), "Failed to set the baudrate!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("control_turn_node"), "Succeeded to set the baudrate.");
    }

    // Use Position Control Mode (value 3 represents position control mode in Dynamixel protocol)
    dxl_comm_result = packetHandler->write1ByteTxRx(
        this->portHandler,
        BROADCAST_ID,  // Send to all connected motors
        ADDR_OPERATING_MODE,
        3,  // Value 3 = Position Control Mode
        &dxl_error
    );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("control_turn_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("control_turn_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL (value 1 = enable torque)
  dxl_comm_result = packetHandler->write1ByteTxRx(
    this->portHandler,
    BROADCAST_ID,  // Send to all connected motors
    ADDR_TORQUE_ENABLE,
    1,  // Value 1 = Enable torque
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("control_turn_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("control_turn_node"), "Succeeded to enable torque.");
  }
}

/**
 * @brief Read and update the current positions of all motors
 * 
 * Iterates through all connected motors and reads their current positions
 */
void ControlTurn::update_present_positions() {
    for (int id = 1; id <= NUM_MOTORS; id++) {
        uint32_t motor_position = 0;
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            (uint8_t) id,  // Using variable id for better flexibility
            ADDR_PRESENT_POSITION,
            &motor_position,
            &dxl_error
        );
        
        if (dxl_comm_result == COMM_SUCCESS) {
            present_positions[id] = motor_position;  // Store position in the class member map
        }
    }
}

/**
 * @brief Implement gradual position transition for smoother movement
 * 
 * Divides the movement into small steps to create a smooth transition
 * between the current position and the target position
 * 
 * @param target_position The final position to move to
 */
void ControlTurn::gradual_transition(int target_position) {
    const int step_size = 60;  /**< Number of intermediate positions for smooth transition */
    const int delay_ms = 15;   /**< Delay between steps in milliseconds */
    
    // Get current position
    update_present_positions();
    int current_position = present_positions[1]; // Using motor ID 1
    
    // Calculate step increment for smooth transition
    float step = static_cast<float>(target_position - current_position) / step_size;
    
    // Execute the gradual transition through multiple smaller steps
    for (int i = 1; i <= step_size; i++) {
        int position = current_position + std::round(step * i);
        
        // Split the position value into 4 bytes as required by Dynamixel protocol
        uint8_t param_goal_position[4];
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position));  // Lower byte of lower word
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position));  // Higher byte of lower word
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position));  // Lower byte of higher word
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position));  // Higher byte of higher word
        
        // Clear previous parameters and add new one
        groupSyncWrite->clearParam();
        if (!groupSyncWrite->addParam(1, param_goal_position)) { // Using motor ID 1
            RCLCPP_WARN(this->get_logger(), "SyncWrite addParam failed");
            continue;
        }
        
        // Send the packet to the motor
        int dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "SyncWrite Failed: %s", 
                         packetHandler->getTxRxResult(dxl_comm_result));
        }
        
        // Short pause between steps for smooth movement
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    
    // Final step to ensure exact target position is reached
    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(target_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(target_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(target_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(target_position));
    
    groupSyncWrite->clearParam();
    groupSyncWrite->addParam(1, param_goal_position);
    groupSyncWrite->txPacket();
}

/**
 * @brief Main function
 * 
 * Initializes ROS, creates the node, and starts the spin loop
 * 
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit status
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS
    rclcpp::spin(std::make_shared<ControlTurn>());  // Create node and start processing
    rclcpp::shutdown();  // Clean shutdown when node is terminated
    return 0;
}