#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
from tf2_ros import TransformListener, Buffer
import math
import time

class NavigationEndToEndTester(Node):
    def __init__(self):
        super().__init__('navigation_e2e_tester')

        # Publishers for sensor inputs
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/data', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.goal_pub = self.create_publisher(MultiGoal, '/waypoint_goals', 10)

        # Subscribers for outputs
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(String, '/nav_status', self.status_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/world_odom', self.odom_callback, 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Navigation End-to-End Tester initialized')

        # Navigation state tracking
        self.navigation_states = []
        self.current_cmd_vel = None
        self.current_odom = None

        # Timer to control test sequence
        self.test_timer = self.create_timer(0.1, self.run_test_sequence)  # 10Hz
        self.test_phase = 0  # 0: init, 1: send GPS/IMU, 2: send goals, 3: monitor, 4: finish
        self.test_time = 0.0

        # Initial GPS position (San Francisco as reference)
        self.initial_lat = 37.7749
        self.initial_lon = -122.4194
        self.initial_alt = 10.0

        # Goal waypoints (small rectangular path for testing)
        self.waypoints = [
            (self.initial_lat + 0.0001, self.initial_lon, self.initial_alt),  # Move North
            (self.initial_lat + 0.0001, self.initial_lon + 0.0001, self.initial_alt),  # Move East
            (self.initial_lat, self.initial_lon + 0.0001, self.initial_alt),  # Move South
            (self.initial_lat, self.initial_lon, self.initial_alt)  # Back to start
        ]

        self.current_waypoint_idx = 0

    def run_test_sequence(self):
        self.test_time += 0.1

        if self.test_phase == 0:  # Initialization
            self.get_logger().info('Phase 0: Initialization')
            self.test_phase = 1
            self.test_time = 0.0

        elif self.test_phase == 1:  # Send GPS/IMU data for a few seconds
            self.publish_sensor_data()

            if self.test_time > 3.0:  # After 3 seconds of sensor data
                self.get_logger().info('Phase 1 complete: Sent sensor data, now sending goals')
                self.test_phase = 2
                self.test_time = 0.0
                self.send_goals()

        elif self.test_phase == 2:  # Monitor navigation progress
            if self.test_time > 1.0:  # After 1 second, check if nav started
                self.get_logger().info('Phase 2: Monitoring navigation progress')
                self.test_phase = 3

        elif self.test_phase == 3:  # Continue monitoring
            self.publish_sensor_data()  # Continue publishing sensor data

            # Check if navigation is progressing
            if len(self.navigation_states) > 0:
                current_state = self.navigation_states[-1]
                self.get_logger().info(f'Current navigation state: {current_state}')

                # If we reached the completed state
                if current_state == 'COMPLETED':
                    self.get_logger().info('SUCCESS: Navigation completed!')
                    self.test_phase = 4

            if self.test_time > 60.0:  # Timeout after 60 seconds
                self.get_logger().info('Test timeout - ending test')
                self.test_phase = 4

        elif self.test_phase == 4:  # Finish test
            self.get_logger().info('Test completed. Final statistics:')
            self.get_logger().info(f'  Total states recorded: {len(self.navigation_states)}')
            if self.navigation_states:
                self.get_logger.info(f'  Final state: {self.navigation_states[-1]}')
            if self.current_cmd_vel:
                self.get_logger.info(f'  Last cmd_vel: vx={self.current_cmd_vel.linear.x:.2f}, '
                                   f'vy={self.current_cmd_vel.linear.y:.2f}, '
                                   f'wz={self.current_cmd_vel.angular.z:.2f}')
            if self.current_odom:
                self.get_logger.info(f'  Last odom pos: x={self.current_odom.pose.pose.position.x:.2f}, '
                                   f'y={self.current_odom.pose.pose.position.y:.2f}')

            # Cancel timer to stop the test
            self.test_timer.cancel()
            self.get_logger().info('End-to-End Test Complete!')

    def publish_sensor_data(self):
        # Publish simulated GPS data (start at initial position and simulate slight movements)
        gps_msg = NavSatFix()
        # Simulate movement based on time - start near initial and gradually move
        time_factor = min(self.test_time / 10.0, 1.0)  # Gradually move from start to first waypoint
        target_lat = self.initial_lat + (self.waypoints[0][0] - self.initial_lat) * time_factor
        target_lon = self.initial_lon + (self.waypoints[0][1] - self.initial_lon) * time_factor
        target_alt = self.initial_alt

        # Add small noise to make it more realistic
        gps_msg.latitude = target_lat + 0.00001 * math.sin(self.test_time)
        gps_msg.longitude = target_lon + 0.00001 * math.cos(self.test_time)
        gps_msg.altitude = target_alt + 0.1 * math.sin(self.test_time * 2)

        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        self.gps_pub.publish(gps_msg)

        # Publish simulated IMU data
        imu_msg = Imu()
        # Start with no rotation, gradually introduce small rotations as robot moves
        imu_msg.orientation.w = 1.0  # Initially no rotation
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.1 * math.sin(self.test_time * 0.5)  # Small oscillating yaw
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'
        self.imu_pub.publish(imu_msg)

    def send_goals(self):
        # Create and send waypoints for navigation
        goal_msg = MultiGoal()

        for i, (lat, lon, alt) in enumerate(self.waypoints):
            wp = MultiGoalPoint()
            wp.x_or_lat = lat
            wp.y_or_lon = lon
            wp.z_or_alt = alt
            wp.yaw = 0.0  # No specific yaw requirement
            wp.vel = 0.5  # Moderate velocity
            goal_msg.multi_goal_points.append(wp)

        goal_msg.is_gps_aid = True  # Using GPS coordinates
        goal_msg.is_gps_hgt = True
        goal_msg.ctl_mode = 0  # vel & yawrate

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Sent {len(goal_msg.multi_goal_points)} waypoints for navigation')

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg
        # Log occasionally to avoid spam
        if int(self.test_time * 10) % 50 == 0:  # Every 5 seconds
            self.get_logger().info(f'Received cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}')

    def status_callback(self, msg):
        # Track navigation states over time
        self.navigation_states.append(msg.data)
        self.get_logger().info(f'Navigation status update: {msg.data}')

    def odom_callback(self, msg):
        self.current_odom = msg
        # Log occasionally to avoid spam
        if int(self.test_time * 10) % 100 == 0:  # Every 10 seconds
            self.get_logger().info(f'Odom update: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')


def main(args=None):
    rclpy.init(args=args)

    tester = NavigationEndToEndTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Shutting down navigation end-to-end tester...')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()