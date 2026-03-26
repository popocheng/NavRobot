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

class WorldOdomNavigationTester(Node):
    def __init__(self):
        super().__init__('world_odom_navigation_tester')

        # Publishers for fused odometry and goals
        self.world_odom_pub = self.create_publisher(Odometry, '/world_odom', 10)
        self.goal_pub = self.create_publisher(MultiGoal, '/waypoint_goals', 10)

        # Subscribers for outputs
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(String, '/nav_status', self.status_callback, 10)

        self.get_logger().info('World Odom Navigation Tester initialized')

        # Navigation state tracking
        self.navigation_states = []
        self.current_cmd_vel = None
        self.test_time = 0.0

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

    def run_test_sequence(self):
        self.test_time += 0.1

        if self.test_phase == 0:  # Initialization
            self.get_logger().info('Phase 0: Initialization')
            self.test_phase = 1
            self.test_time = 0.0

        elif self.test_phase == 1:  # Send initial world_odom data
            self.publish_world_odom()

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
            self.publish_world_odom()  # Continue publishing odometry as robot moves

            # Check if navigation is progressing
            if len(self.navigation_states) > 0:
                current_state = self.navigation_states[-1]
                self.get_logger().info_throttle(5.0, f'Current navigation state: {current_state}')

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
                self.get_logger.info(f'  Final state: {self.navigation_states[-1]}')
            if self.current_cmd_vel:
                self.get_logger.info(f'  Last cmd_vel: vx={self.current_cmd_vel.linear.x:.2f}, '
                                   f'vy={self.current_cmd_vel.linear.y:.2f}, '
                                   f'wz={self.current_cmd_vel.angular.z:.2f}')

            # Cancel timer to stop the test
            self.test_timer.cancel()
            self.get_logger().info('World Odom Navigation Test Complete!')

    def publish_world_odom(self):
        # Create and publish world_odom message simulating robot movement
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'robot_base'

        # Simulate robot moving along the path based on test time
        # For simplicity, we'll interpolate between waypoints over time
        total_duration = 100.0  # seconds for entire path
        progress = min(self.test_time / total_duration, 1.0)

        # Find current segment based on progress
        num_segments = len(self.waypoints)
        segment_progress = progress * num_segments
        current_segment = min(int(segment_progress), num_segments - 1)
        local_progress = segment_progress - current_segment

        if current_segment < num_segments - 1:
            # Interpolate between current waypoint and next
            start_wp = self.waypoints[current_segment]
            end_wp = self.waypoints[current_segment + 1]

            x = start_wp[0] + (end_wp[0] - start_wp[0]) * min(local_progress, 1.0)
            y = start_wp[1] + (end_wp[1] - start_wp[1]) * min(local_progress, 1.0)
        else:
            # At final waypoint
            x = self.waypoints[-1][0]
            y = self.waypoints[-1][1]

        # Add some movement to simulate robot approaching goals
        # Adjust based on current goal
        current_goal = self.waypoints[min(self.current_waypoint_idx, len(self.waypoints)-1)]
        dx = current_goal[0] - x
        dy = current_goal[1] - y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)

        # When close to goal, slow movement
        if distance_to_goal < 0.5:
            # Hold position near goal
            target_x = current_goal[0] + 0.1 * math.sin(self.test_time)
            target_y = current_goal[1] + 0.1 * math.cos(self.test_time)
        else:
            # Move toward goal with some error simulation
            target_x = x + dx * 0.1  # Move partway toward goal
            target_y = y + dy * 0.1

        odom_msg.pose.pose.position.x = target_x
        odom_msg.pose.pose.position.y = target_y
        odom_msg.pose.pose.position.z = 0.0

        # Calculate yaw to face approximately in direction of motion
        yaw = math.atan2(dy, dx) if distance_to_goal > 0.1 else self.start_yaw
        odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Velocity information (simulated)
        odom_msg.twist.twist.linear.x = 0.5  # Some forward velocity when moving
        odom_msg.twist.twist.angular.z = 0.2  # Some angular velocity when turning

        self.world_odom_pub.publish(odom_msg)

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

        goal_msg.is_gps_aid = False  # Using local coordinates now
        goal_msg.is_gps_hgt = False
        goal_msg.ctl_mode = 0  # vel & yawrate

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Sent {len(goal_msg.multi_goal_points)} local waypoints for navigation')

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg
        # Log occasionally to avoid spam
        if int(self.test_time * 10) % 20 == 0:  # Every 2 seconds
            self.get_logger().info_throttle(2.0, f'Received cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}')

    def status_callback(self, msg):
        # Track navigation states over time
        self.navigation_states.append(msg.data)
        self.get_logger().info_throttle(1.0, f'Navigation status update: {msg.data}')


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