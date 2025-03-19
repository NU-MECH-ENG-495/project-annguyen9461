#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
from std_msgs.msg import String
from std_srvs.srv import Empty

arduino_comm = serial.Serial()

class InoComm(Node):
    """
    ROS2 Node for serial communication and service interaction with the Arduino

    Responsibilities:
    - Communicates with an Arduino device over a serial port.

    **Services**:

    **Clients**:

    """

    def __init__(self):
        """
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

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

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
    # import sys
    # main(args=sys.argv)
    main()
