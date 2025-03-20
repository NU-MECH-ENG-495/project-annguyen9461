#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from claybot_interfaces.srv import SendMoveCommand
import sys

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

def main(args=None):
    rclpy.init(args=args)
    client = Client()
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
    else:
        command = 'u'  # default to "up" if no argument provided
    
    print(f"Sending command: {command}")
    response = client.send_request(command)
    print("Response received")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)