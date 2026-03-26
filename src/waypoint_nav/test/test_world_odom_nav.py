#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
import math
import time
from datetime import datetime, timedelta

class WorldOdomNavigationTester(Node):
    def __init__(self):
        super().__init__('world_odom_navigation_tester')

        self.goal_pub = self.create_publisher(MultiGoal, '/waypoint_goals', 10)

        # Subscribers for outputs
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(String, '/nav_status', self.status_callback, 10)

        self.get_logger().info('World Odom Navigation Tester initialized')

        # Navigation state tracking
        self.navigation_states = []
        self.current_cmd_vel = None
        self.test_time = 0.0

        # Time tracking for throttling messages
        self.last_status_log_time = {}
        self.last_cmd_vel_log_time = datetime.now()

        # Timer to control test sequence
        self.test_timer = self.create_timer(0.1, self.run_test_sequence)  # 10Hz
        self.test_phase = 0  # 0: init, 1: send world_odom, 2: send goals, 3: monitor, 4: finish

        # Starting position (origin)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        # Goal waypoints (square path for testing)
        self.waypoints = [
            (2.0, 0.0),    # 2m east
            (2.0, 2.0),    # 2m north
            (0.0, 2.0),    # 2m west
            (0.0, 0.0)     # back to origin
        ]

        self.current_waypoint_idx = 0

    def should_log_throttled(self, key, interval_seconds):
        """Helper method to throttle log messages"""
        now = datetime.now()
        if key not in self.last_status_log_time:
            self.last_status_log_time[key] = now
            return True

        if (now - self.last_status_log_time[key]).total_seconds() >= interval_seconds:
            self.last_status_log_time[key] = now
            return True
        return False

    def run_test_sequence(self):
        self.test_time += 0.1

        if self.test_phase == 0:  # Initialization
            self.get_logger().info('Phase 0: Initialization')
            self.test_phase = 1
            self.test_time = 0.0

        elif self.test_phase == 1:  # Send initial world_odom data

            if self.test_time > 2.0:  # After 2 seconds of odometry data
                self.get_logger().info('Phase 1 complete: Published initial odometry')
                self.test_phase = 2
                self.test_time = 0.0
                self.send_goals()

        elif self.test_phase == 2:  # Wait a bit after sending goals
            if self.test_time > 1.0:
                self.get_logger().info('Phase 2: Goals sent, starting navigation')
                self.test_phase = 3
                self.test_time = 0.0

        elif self.test_phase == 3:  # Monitor navigation progress

            # Check if navigation is progressing
            if len(self.navigation_states) > 0:
                current_state = self.navigation_states[-1]

                # Throttle the logging to once every 5 seconds
                if self.should_log_throttled('navigation_state', 5.0):
                    self.get_logger().info(f'Current navigation state: {current_state}')

                # If we reached the completed state
                if current_state == 'COMPLETED':
                    self.get_logger().info('SUCCESS: Navigation completed!')
                    self.test_phase = 4

            if self.test_time > 120.0:  # Timeout after 120 seconds
                self.get_logger().info('Test timeout - ending test')
                self.test_phase = 4

        elif self.test_phase == 4:  # Finish test
            self.get_logger().info('Test completed. Final statistics:')
            self.get_logger().info(f'  Total states recorded: {len(self.navigation_states)}')
            if self.navigation_states:
                self.get_logger().info(f'  Final state: {self.navigation_states[-1]}')
            if self.current_cmd_vel:
                self.get_logger().info(f'  Last cmd_vel: vx={self.current_cmd_vel.linear.x:.2f}, '
                                   f'vy={self.current_cmd_vel.linear.y:.2f}, '
                                   f'wz={self.current_cmd_vel.angular.z:.2f}')

            # Cancel timer to stop the test
            self.test_timer.cancel()
            self.get_logger().info('World Odom Navigation Test Complete!')

    def send_goals(self):
        # Create and send waypoints for navigation
        goal_msg = MultiGoal()

        for i, (x, y) in enumerate(self.waypoints):
            wp = MultiGoalPoint()
            wp.x_or_lat = x  # Using local coordinates now instead of GPS
            wp.y_or_lon = y
            wp.z_or_alt = 0.0
            wp.yaw = 0.0  # No specific yaw requirement
            wp.vel = 0.5  # Moderate velocity
            goal_msg.multi_goal_points.append(wp)

        goal_msg.is_gps_aid = True  # Using local coordinates now
        goal_msg.is_gps_hgt = False
        goal_msg.ctl_mode = 0  # vel & yawrate

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Sent {len(goal_msg.multi_goal_points)} local waypoints for navigation')

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg
        # Log occasionally to avoid spam
        now = datetime.now()
        if (now - self.last_cmd_vel_log_time).total_seconds() >= 2.0:  # Every 2 seconds
            self.get_logger().info(f'Received cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}')
            self.last_cmd_vel_log_time = now

    def status_callback(self, msg):
        # Track navigation states over time
        self.navigation_states.append(msg.data)
        # Throttle the logging to once every 1 second
        if self.should_log_throttled('status_update', 1.0):
            self.get_logger().info(f'Navigation status update: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    tester = WorldOdomNavigationTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Shutting down world odom navigation tester...')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()