#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
import math
import time

class WaypointNavTester(Node):
    def __init__(self):
        super().__init__('waypoint_nav_tester')

        # Publishers for sensor inputs
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/data', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.goal_pub = self.create_publisher(MultiGoal, '/waypoint_goals', 10)

        # Subscriber for command velocity
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Subscriber for navigation status
        self.status_sub = self.create_subscription(String, '/nav_status', self.status_callback, 10)

        self.get_logger().info('Waypoint Navigation Tester initialized')

        # Timer to periodically publish sensor data
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10Hz
        self.time_counter = 0

        # Flag to track if goals have been published
        self.goals_published = False

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}')

    def status_callback(self, msg):
        self.get_logger().info(f'Navigation status: {msg.data}')

    def publish_test_data(self):
        self.time_counter += 0.1

        # Publish simulated GPS data (starting at a fixed point)
        gps_msg = NavSatFix()
        gps_msg.latitude = 37.7749 + 0.00001 * self.time_counter  # Start from a point
        gps_msg.longitude = -122.4194 + 0.00001 * self.time_counter
        gps_msg.altitude = 10.0
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        self.gps_pub.publish(gps_msg)

        # Publish simulated IMU data
        imu_msg = Imu()
        # Initialize with identity quaternion (no rotation)
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'
        self.imu_pub.publish(imu_msg)

        # Publish waypoints once after initialization delay
        if not self.goals_published and self.time_counter > 2.0:  # After 2 seconds
            self.publish_waypoints()
            self.goals_published = True

    def publish_waypoints(self):
        # Create a sequence of waypoints
        goal_msg = MultiGoal()

        # Add a few test waypoints (these would normally be in GPS coordinates)
        # Create a small square pattern for testing
        base_lat = 37.7749
        base_lon = -122.4194

        for i in range(4):
            wp = MultiGoalPoint()
            if i == 0:
                wp.x_or_lat = base_lat + 0.001  # Move North
                wp.y_or_lon = base_lon
            elif i == 1:
                wp.x_or_lat = base_lat + 0.001  # Move East
                wp.y_or_lon = base_lon + 0.001
            elif i == 2:
                wp.x_or_lat = base_lat  # Move South
                wp.y_or_lon = base_lon + 0.001
            elif i == 3:
                wp.x_or_lat = base_lat  # Move West back to start
                wp.y_or_lon = base_lon

            wp.z_or_alt = 10.0
            wp.yaw = 0.0  # No specific yaw requirement
            wp.vel = 1.0  # Max velocity 1 m/s
            goal_msg.multi_goal_points.append(wp)

        goal_msg.is_gps_aid = True  # Using GPS coordinates
        goal_msg.is_gps_hgt = True
        goal_msg.ctl_mode = 0  # vel & yawrate

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Published {len(goal_msg.multi_goal_points)} waypoints for navigation')


def main(args=None):
    rclpy.init(args=args)

    tester = WaypointNavTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Shutting down waypoint navigation tester...')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()