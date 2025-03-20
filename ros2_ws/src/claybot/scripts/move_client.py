#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from claybot_interfaces.srv import SendMoveCommand
import sys
import time

class Client(Node):
    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(SendMoveCommand, 'send_move_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SendMoveCommand.Request()

    def send_request(self, command_str):
        self.req.command = String()
        self.req.command.data = command_str
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_command_sequence(self, commands, delay_between_commands=0.5):
        """
        Send a sequence of commands with delays between them
        
        :param commands: List of command characters ('u', 'd', 'p')
        :param delay_between_commands: Delay in seconds between commands

        Examples:
        ros2 run claybot move_client.py
        ros2 run claybot move_client.py uudd
        """
        for i, cmd in enumerate(commands):
            if cmd not in ['u', 'd', 'p']:
                self.get_logger().warn(f"Invalid command '{cmd}' - skipping")
                continue
            
            self.get_logger().info(f"Command {i+1}/{len(commands)}: {cmd}")
            self.send_request(cmd)

            # Don't delay after the last command
            if i < len(commands) - 1:
                self.get_logger().info(f"Waiting {delay_between_commands} seconds before next command...")
                time.sleep(delay_between_commands)

def main(args=None):
    rclpy.init(args=args)
    client = Client()
    
    # Default command sequence
    default_sequence = ['u', 'u', 'd', 'd']
    
    # Check if command sequence was provided as argument
    if len(sys.argv) > 1:
        # Convert input string to list of commands
        # For example: "uudd" becomes ['u', 'u', 'd', 'd']
        command_sequence = list(sys.argv[1])
    else:
        command_sequence = default_sequence
    
    print(f"Preparing to send command sequence: {command_sequence}")
    
    # Send the sequence
    client.send_command_sequence(command_sequence)
    
    print("Command sequence completed")
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)