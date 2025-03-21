#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from claybot_interfaces.srv import SendMoveCommand, SendTurnCommand, CutShape
from claybot_interfaces.msg import RobotState, SetPosition
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
        self.set_position_pub = self.create_publisher(SetPosition, 'set_position', 10)

        # Define delay times (in seconds)
        self.delay_after_down = 0.1  # Pause after moving down
        self.delay_after_up = 1.0    # Pause after moving up, before turning
        self.delay_after_turn = 0.5  # Pause after turning
        
        self.get_logger().info('Cut Shape Service initialized')

    def publish_turn(self, degrees):
        # Clamp degrees to [0, 360)
        while degrees < 0:
            degrees += 360
        while degrees >= 360:
            degrees -= 360

        # Convert to position (0–4096)
        position = int((degrees / 360.0) * 4096)

        # Publish to /set_position topic
        msg = SetPosition()
        msg.id = 1  # Update with actual motor ID!
        msg.position = position
        self.set_position_pub.publish(msg)
        self.get_logger().info(f"Sent turn command: {degrees}° → pos {position}")

    def cut_shape_callback(self, request, response):
        self.get_logger().info(f'Cutting shape: {request.shape_type}')
        
        num_turns = 1
        shape_degree = 360 / num_turns

        if request.shape_type == 'squ':
            num_turns = 4
        elif request.shape_type == 'tri':
            num_turns = 3
        elif request.shape_type == 'hex':
            num_turns = 6
        else:
            self.get_logger().error(f"Unknown shape type: {request.shape_type}")
            response.success = False
            response.message = f"Unknown shape type: {request.shape_type}"
            return response
        
        shape_degree = 360 / num_turns
        for i in range(num_turns):
            self.call_move_vertical_service("d")
            time.sleep(self.delay_after_down)
            self.call_move_vertical_service("u")
            time.sleep(self.delay_after_up)
            turn_degree = shape_degree * i
            self.publish_turn(turn_degree)
            time.sleep(self.delay_after_turn)
        
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

def main(args=None):
    rclpy.init(args=args)
    node = CutShapeService()
    rclpy.spin(node)

if __name__ == '__main__':
    main()