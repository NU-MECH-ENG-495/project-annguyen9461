#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from claybot_interfaces.srv import SendMoveCommand, SendTurnCommand, CutShape
from claybot_interfaces.msg import RobotState
from std_msgs.msg import String
import time

class CutShapeService(Node):
    """
    To automatically send movement commands when cutting shapes.

    ros2 service call /cut_shape claybot_interfaces/srv/CutShape "{shape_type: 'SQUARE'}"
    """
    def __init__(self):
        super().__init__('cut_shape_service')
        self.srv = self.create_service(
            CutShape,
            'cut_shape',
            self.cut_shape_callback)
        
        # Create client for move UP/DOWN command
        self.move_client = self.create_client(SendMoveCommand, 'send_move_command')
        self.turn_client = self.create_client(SendTurnCommand, 'send_turn_command')

    def cut_shape_callback(self, request, response):
        self.get_logger().info(f'Cutting shape: {request.shape_type}')
        
        # Call other services as needed
        if request.shape_type == 'SQUARE':
            for _ in range(4):
                self.call_move_vertical_service("d")
                self.call_move_vertical_service("u")
                self.call_turn_service(90.0)

        elif request.shape_type == 'TRIANGLE':
            for _ in range(3):
                self.call_move_vertical_service("d")
                self.call_move_vertical_service("u")
                self.call_turn_service(120.0)

        elif request.shape_type == 'HEXAGON':
            for _ in range(6):
                self.call_move_vertical_service("d")
                self.call_move_vertical_service("u")
                self.call_turn_service(60.0)

        else:
            self.get_logger().error(f"Unknown shape type: {request.shape_type}")
            response.success = False
            response.message = f"Unknown shape type: {request.shape_type}"
            return response
            
        # Set response
        response.success = True
        response.message = f"Successfully cut {request.shape_type}"
        return response
        
    def call_move_vertical_service(self, move_type):
        # Create and send the request
        request = SendMoveCommand.Request()
        request.command = String(data=move_type)
        
        # Wait for service to be available
        if not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Move service not available')
            return False
            
        # Call the service
        future = self.move_client.call_async(request)
        # Wait for the future to complete with timeout
        timeout = 2.0  # seconds
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f'Move {move_type} completed successfully')
                    return True
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {str(e)}')
                    return False
        
        self.get_logger().error(f'Service call timeout ({timeout} seconds)')
        return False

    def call_turn_service(self, angle_deg):
        # self.get_logger().info(f'[Placeholder] Turning robot by {angle_deg} degrees.')
        # time.sleep(0.5)  # Simulate a brief delay

        # Create and send the request
        request = SendTurnCommand.Request()
        request.degrees = angle_deg
        
        # Wait for service to be available
        if not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Move service not available')
            return False
            
        # Call the service
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
        
def main(args=None):
    rclpy.init(args=args)
    node = CutShapeService()
    rclpy.spin(node)

if __name__ == '__main__':
    main()