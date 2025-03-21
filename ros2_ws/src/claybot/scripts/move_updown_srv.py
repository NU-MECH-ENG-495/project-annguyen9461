#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
from std_msgs.msg import String
from std_srvs.srv import Empty
from claybot_interfaces.srv import SendMoveCommand
import sys
import time

arduino_comm = serial.Serial()

class InoComm(Node):
    """
    ROS2 Node for serial communication and service interaction with the Arduino

    Responsibilities:
    - Communicates with an Arduino device over a serial port.
    """

    def __init__(self):
        """
        ros2 service call /send_move_command claybot_interfaces/srv/SendMoveCommand "{command: {data: 'u'}}"

        Initialize the InoComm node, sets up serial comm, and creates ROS2 service and clients.

        :arg None: No arguments are passed during initialization.
        :type None: NoneType

        :raises RuntimeError: If the Arduino cannot be found during serial port discovery.
        """
        super().__init__('inocomm')

        ARDUINO_SERIAL = '24336303633351E0D100'

        # ports discovery
        ports = list(list_ports.comports(True))  # get list of ports
        arduino_port = None

        # grab the port that's connected to the Arduino
        for p in ports:
            if p.serial_number == ARDUINO_SERIAL:
                arduino_port = p.device
                break

        # check Arduino has been found
        if arduino_port is None:
            self.get_logger().error('error finding arduino')
        else:
            self.get_logger().info('found arduino')

        self.connect_serial_port(serial_port=arduino_port, baud_rate=115200)

        self.move_srv = self.create_service(SendMoveCommand, 'send_move_command', self.move_callback)

    def move_callback(self, request, response):
        """
        From terminal:
        ros2 service call /send_move_command claybot_interfaces/srv/SendMoveCommand "{command: {data: 'u'}}"
        """
        if(request.command.data == 'u'):
            msg = String()
            msg.data = 'u'
            self.write_serial_data(msg)
            self.get_logger().info('Moving Up')
        elif(request.command.data == 'd'):
            msg = String()
            msg.data = 'd'
            self.write_serial_data(msg)
            self.get_logger().info('Moving Down')
        elif(request.command.data == 'p'):
            msg = String()
            msg.data = 'p'
            self.write_serial_data(msg)
            self.get_logger().info('Paused')  
        else:
            self.get_logger().error("No valid string option sent. Send u (up), d (down), or p (pause).")
            return response

        serial_val = self.read_serial_data_with_timeout()
        self.get_logger().info('Movement Done')
        
        # Always return the response to complete the service call
        return response

    def write_serial_data(self, msg: String):
        try:
            msg_str = str(msg.data)
            self.get_logger().info(f'Writing to serial: {msg_str}')
            arduino_comm.write(msg_str.encode("utf-8"))
            # Flush to ensure data is sent immediately
            arduino_comm.flush()
        except Exception as e:
            self.get_logger().error(f'Cannot write serial data!: {e}')

    def read_serial_data_with_timeout(self, timeout=5.0):
        """
        Read serial data from the Arduino device with a timeout.
        
        :param timeout: Maximum time (in seconds) to wait for a response
        :return: A message containing the valid serial data or None if timeout
        """
        try:
            start_time = time.time()
            valid_responses = ["Moving Up", "Moving Down", "Paused", "Resumed", "Auto-unpaused"]
            
            while (time.time() - start_time) < timeout:
                if arduino_comm.in_waiting > 0:
                    data = arduino_comm.readline().decode('utf-8').rstrip('\n').rstrip('\r')
                    self.get_logger().info(f'Received data: {data}')
                    
                    # Check if we have any valid response
                    for response in valid_responses:
                        if response in data:
                            msg = String()
                            msg.data = data
                            return msg
                
                # Small sleep to prevent CPU hogging
                time.sleep(0.1)
            
            self.get_logger().warn('Timeout waiting for Arduino response')
            return None
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            self.get_logger().error('Cannot read serial data!')
            return None

    def connect_serial_port(self, serial_port, baud_rate):
        """
        Configure and opens the serial port for communication with the Arduino device.

        :arg serial_port: The name of the serial port to connect to.
        :type serial_port: str
        :arg baud_rate: The baud rate for serial communication.
        :type baud_rate: int
        """
        arduino_comm.port = serial_port
        arduino_comm.baudrate = baud_rate
        arduino_comm.timeout = 1
        arduino_comm.open()

def main(args=None):
    """
    Entry point for starting the InoComm node.

    :arg args: Optional command-line arguments passed to the ROS2 program.
    :type args: list or NoneType
    """
    rclpy.init(args=args)
    node = InoComm()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
