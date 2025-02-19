#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import TrackDetection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class TrackletVisualizer(Node):
    def __init__(self):
        super().__init__('tracklet_visualizer')
        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            '/color/yolov4_Spatial_tracklets',
            self.tracklet_callback,
            10)
        self.publisher = self.create_publisher(MarkerArray, '/tracked_objects_markers', 10)
        self.get_logger().info("Tracklet Visualizer Node Started")

    def tracklet_callback(self, msg):
        self.get_logger().info(f"Received {len(msg.detections)} detections")
        
        marker_array = MarkerArray()

        if len(msg.detections) == 0:
            self.get_logger().warn("No detections received!")

        for i, detection in enumerate(msg.detections):
            if len(detection.results) == 0:
                self.get_logger().warn(f"Detection {i} has no results")
                continue

            position = detection.results[0].pose.pose.position

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose.position.x = position.x
            marker.pose.position.y = position.y
            marker.pose.position.z = position.z
            marker.id = i

            marker_array.markers.append(marker)

        self.get_logger().info(f"Publishing {len(marker_array.markers)} markers")
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TrackletVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
