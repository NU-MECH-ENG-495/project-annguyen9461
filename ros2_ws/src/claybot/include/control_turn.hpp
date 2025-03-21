#ifndef CONTROL_TURN_HPP_
#define CONTROL_TURN_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "claybot_interfaces/msg/set_position.hpp"
#include "claybot_interfaces/srv/get_position.hpp"

class ControlTurn : public rclcpp::Node 
{
public:
    using SetPosition = claybot_interfaces::msg::SetPosition;
    using GetPosition = claybot_interfaces::srv::GetPosition;

    ControlTurn();
    virtual ~ControlTurn();

private:
    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler;
    dynamixel::GroupSyncWrite* groupSyncWrite;

    void initDynamixels();

    rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
    rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

    int present_position;
};

#endif  // CONTROL_TURN_HPP_