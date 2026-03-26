#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from nav_msgs.msg import Odometry
from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
import math
import time

class WaypointNavTester(Node):
    def __init__(self):
        super().__init__('waypoint_nav_tester')

        # Publishers for fused odometry and goals
        self.world_odom_pub = self.create_publisher(Odometry, '/world_odom', 10)
        self.goal_pub = self.create_publisher(MultiGoal, '/waypoint_goals', 10)

        # Subscriber for command velocity
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Subscriber for navigation status
        self.status_sub = self.create_subscription(String, '/nav_status', self.status_callback, 10)

        self.get_logger().info('Waypoint Navigation Tester initialized (updated for world_odom)')

        # Timer to periodically publish fused odometry data
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10Hz
        self.time_counter = 0

        # Flag to track if goals have been published
        self.goals_published = False

        # Robot starting position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}')

    def status_callback(self, msg):
        self.get_logger().info(f'Navigation status: {msg.data}')

    def publish_test_data(self):
        self.time_counter += 0.1

        # Publish simulated world_odom data (simulating robot movement)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'robot_base'

        # Simulate robot movement based on time - follow a simple path
        # This simulates the robot's position based on cmd_vel commands
        if hasattr(self, 'last_cmd_vel'):
            dt = 0.1  # Time step
            self.robot_x += self.last_cmd_vel.linear.x * math.cos(self.robot_yaw) * dt
            self.robot_y += self.last_cmd_vel.linear.x * math.sin(self.robot_yaw) * dt
            self.robot_yaw += self.last_cmd_vel.angular.z * dt

        # Add small deviations to simulate real movement
        self.robot_x += 0.001 * math.sin(self.time_counter)
        self.robot_y += 0.001 * math.cos(self.time_counter)

        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation from yaw
        odom_msg.pose.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.robot_yaw / 2.0)

        # Set twist (simulated based on last command)
        if hasattr(self, 'last_cmd_vel'):
            odom_msg.twist.twist.linear.x = self.last_cmd_vel.linear.x
            odom_msg.twist.twist.angular.z = self.last_cmd_vel.angular.z

        self.world_odom_pub.publish(odom_msg)

        # Publish waypoints once after initialization delay
        if not self.goals_published and self.time_counter > 2.0:  # After 2 seconds
            self.publish_waypoints()
            self.goals_published = True

    def publish_waypoints(self):
        # Create a sequence of waypoints in local coordinates (since we're using world_odom)
        goal_msg = MultiGoal()

        # Add a few test waypoints (these are now in local coordinates relative to world origin)
        # Create a small square pattern for testing
        waypoints = [
            (2.0, 0.0),    # 2m east
            (2.0, 2.0),    # 2m north
            (0.0, 2.0),    # 2m west
            (0.0, 0.0)     # back to origin
        ]

        for x, y in waypoints:
            wp = MultiGoalPoint()
            wp.x_or_lat = x
            wp.y_or_lon = y
            wp.z_or_alt = 0.0
            wp.yaw = 0.0  # No specific yaw requirement
            wp.vel = 0.5  # Moderate velocity
            goal_msg.multi_goal_points.append(wp)

        goal_msg.is_gps_aid = False  # Now using local coordinates
        goal_msg.is_gps_hgt = False
        goal_msg.ctl_mode = 0  # vel & yawrate

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Published {len(goal_msg.multi_goal_points)} local waypoints for navigation')


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