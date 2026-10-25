#!/usr/bin/env python3
"""
Initial Spin Node - Rotates robot using cmd_vel and moves in safest direction based on laser scan
Continues moving until map is built
Listens to reset_command topic to re-execute after Nav2 restart
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from cure_explore_interfaces.msg import ResetCommand
import math
import numpy as np

# ResetCommand command values
RESET_CMD_SHUTDOWN = 0
RESET_CMD_STARTUP = 1
RESET_CMD_RESTART = 2


class InitialSpinNode(Node):
    def __init__(self):
        super().__init__('initial_spin_node')
        
        # Declare parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('startup_delay', 3.0)  # Wait 3s for Nav2 to fully start
        self.declare_parameter('rotation_speed', 0.5)  # Rotation speed rad/s
        self.declare_parameter('rotation_angle', 1.57)  # 90 degrees
        self.declare_parameter('linear_speed', 0.2)  # Linear movement speed m/s
        self.declare_parameter('move_distance', 0.5)  # Move 0.5m after spin
        self.declare_parameter('safe_cone_angle', 0.785)  # 45 degrees cone (rad)
        self.declare_parameter('min_obstacle_distance', 1.0)  # Minimum 1m clearance
        self.declare_parameter('max_move_attempts', 10)  # Maximum moves if map stays empty
        
        self.robot_id = self.get_parameter('robot_id').value
        self.startup_delay = self.get_parameter('startup_delay').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.move_distance = self.get_parameter('move_distance').value
        self.safe_cone_angle = self.get_parameter('safe_cone_angle').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.max_move_attempts = self.get_parameter('max_move_attempts').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to reset command
        reset_topic = f'/robot_{self.robot_id-1}/reset_command'
        self.reset_sub = self.create_subscription(
            ResetCommand,
            reset_topic,
            self.reset_callback,
            10
        )
        self.get_logger().info(f'Subscribed to reset commands: {reset_topic}')
        self.get_logger().info(f'Subscribed to reset commands: {reset_topic}')
        
        # Subscribe to laser scan
        self.latest_scan = None
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Subscribe to map to check if it's empty
        self.latest_map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # State tracking
        self.move_count = 0
        self.rotation_completed = False
        self.rotation_timer = None
        
        self.get_logger().info(f'Initial Spin Node started. Waiting for STARTUP reset command or initial startup delay...')
        
        # Wait for startup, then execute rotation (for first launch only)
        self.timer = self.create_timer(self.startup_delay, self.execute_initial_rotation)
    
    def reset_callback(self, msg: ResetCommand):
        """Handle reset command - re-execute initialization on STARTUP"""
        cmd_names = {0: 'SHUTDOWN', 1: 'STARTUP', 2: 'RESTART'}
        cmd_name = cmd_names.get(msg.command, f'UNKNOWN({msg.command})')
        
        self.get_logger().info(f'Received reset command: {cmd_name} (code={msg.command})')
        
        if msg.command == RESET_CMD_STARTUP:
            # Reset state
            self.move_count = 0
            self.rotation_completed = False
            
            self.get_logger().info(f'STARTUP command received! Will execute rotation after {self.startup_delay}s delay...')
            
            # Cancel any existing timers
            if hasattr(self, 'timer') and self.timer is not None:
                self.timer.cancel()
            if hasattr(self, '_check_map_timer') and self._check_map_timer is not None:
                self._check_map_timer.cancel()
            if hasattr(self, '_retry_timer') and self._retry_timer is not None:
                self._retry_timer.cancel()
            
            # Schedule new rotation after delay
            self.timer = self.create_timer(self.startup_delay, self.execute_initial_rotation)
    
    def scan_callback(self, msg):
        """Store latest laser scan data"""
        self.latest_scan = msg
    
    def map_callback(self, msg):
        """Store latest map data"""
        self.latest_map = msg
    
    def is_map_empty(self):
        """
        Check if the map is empty or contains valid obstacle data
        Returns: True if map is empty/invalid, False if map has content
        """
        if self.latest_map is None:
            self.get_logger().info('Map data not received yet')
            return True
        
        map_data = self.latest_map
        
        # Check if map has zero size
        if map_data.info.width == 0 or map_data.info.height == 0:
            self.get_logger().info(f'Map has zero size: {map_data.info.width}x{map_data.info.height}')
            return True
        
        # Check if map data is all unknown (-1) or free (0)
        occupied_cells = sum(1 for cell in map_data.data if cell > 50)  # Occupied threshold
        known_cells = sum(1 for cell in map_data.data if cell >= 0)  # Known cells (not -1)
        total_cells = len(map_data.data)
        
        occupied_ratio = occupied_cells / total_cells if total_cells > 0 else 0
        known_ratio = known_cells / total_cells if total_cells > 0 else 0
        
        self.get_logger().info(
            f'Map status: {map_data.info.width}x{map_data.info.height}, '
            f'occupied: {occupied_cells}/{total_cells} ({occupied_ratio*100:.1f}%), '
            f'known: {known_cells}/{total_cells} ({known_ratio*100:.1f}%)'
        )
        
        # Map is considered valid if it has at least 1% known cells or some obstacles
        is_empty = known_ratio < 0.01 and occupied_cells < 10
        
        if is_empty:
            self.get_logger().warn('Map is still empty or has insufficient data')
        else:
            self.get_logger().info('Map has valid data!')
        
        return is_empty
    
    def find_safest_direction(self):
        """
        Find the safest direction based on laser scan data only
        45-degree cone with maximum clearance
        Returns: angle in radians relative to robot's current heading
        """
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available, defaulting to forward direction')
            return 0.0
        
        scan = self.latest_scan
        num_points = len(scan.ranges)
        
        if num_points == 0:
            self.get_logger().warn('Scan has no data points')
            return 0.0
        
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # Calculate cone size in number of scan points
        cone_half_points = int(self.safe_cone_angle / 2.0 / angle_increment)
        
        best_direction = 0.0
        max_min_distance = 0.0
        
        # Scan through all possible directions
        for center_idx in range(num_points):
            # Get cone range
            start_idx = max(0, center_idx - cone_half_points)
            end_idx = min(num_points - 1, center_idx + cone_half_points)
            
            # Find minimum distance in this cone
            cone_ranges = scan.ranges[start_idx:end_idx+1]
            valid_ranges = [r for r in cone_ranges if scan.range_min < r < scan.range_max]
            
            if not valid_ranges:
                continue
            
            min_distance_in_cone = min(valid_ranges)
            
            # Update best direction if this cone has better clearance
            if min_distance_in_cone > max_min_distance:
                max_min_distance = min_distance_in_cone
                best_direction = angle_min + center_idx * angle_increment
        
        self.get_logger().info(f'Safest direction: {math.degrees(best_direction):.1f}° with {max_min_distance:.2f}m clearance')
        
        # Only move if we have sufficient clearance
        if max_min_distance < self.min_obstacle_distance:
            self.get_logger().warn(f'Insufficient clearance ({max_min_distance:.2f}m < {self.min_obstacle_distance}m), skipping move')
            return None
        
        return best_direction
    
    def execute_initial_rotation(self):
        """Execute rotation using cmd_vel"""
        self.timer.cancel()
        
        self.get_logger().info(f'Executing rotation of {math.degrees(self.rotation_angle):.1f}° using cmd_vel...')
        
        # Calculate rotation duration
        rotation_duration = abs(self.rotation_angle / self.rotation_speed)
        
        # Publish rotation command
        twist = Twist()
        twist.angular.z = self.rotation_speed if self.rotation_angle > 0 else -self.rotation_speed
        
        # Publish for the duration
        rate = self.create_rate(10)  # 10 Hz
        start_time = self.get_clock().now()
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < rotation_duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('Rotation completed! Now checking map and finding safest direction...')
        self.rotation_completed = True
        
        # Wait a bit for scan data to settle
        timer = self.create_timer(1.0, self.check_map_and_move)
        self._check_map_timer = timer
    
    def check_map_and_move(self):
        """Check if map is empty and decide whether to move"""
        # Cancel the timer if it exists
        if hasattr(self, '_check_map_timer'):
            self._check_map_timer.cancel()
        
        if not self.rotation_completed:
            return
        
        # Check if map has valid data
        map_is_empty = self.is_map_empty()
        
        if not map_is_empty:
            self.get_logger().info('Map has valid data, initial setup complete!')
            return
        
        # Map is still empty, check move attempts
        if self.move_count >= self.max_move_attempts:
            self.get_logger().warn(f'Reached max move attempts ({self.max_move_attempts}), stopping')
            return
        
        self.get_logger().info(f'Map is empty, initiating move {self.move_count + 1}/{self.max_move_attempts}')
        self.execute_safe_move()
    
    def execute_safe_move(self):
        """Move in the safest direction using cmd_vel (omnidirectional movement)"""
        safe_angle = self.find_safest_direction()
        
        if safe_angle is None:
            self.get_logger().warn('No safe direction found, will retry after delay')
            timer = self.create_timer(2.0, self._retry_check_map)
            self._retry_timer = timer
            return
        
        self.move_count += 1
        self.get_logger().info(f'Moving {self.move_distance}m in direction {math.degrees(safe_angle):.1f}° using omnidirectional cmd_vel (attempt {self.move_count})...')
        
        # Move directly in the safe direction (no rotation needed for omnidirectional robot)
        self._move_in_direction(safe_angle, self.move_distance)
        
        self.get_logger().info(f'Move {self.move_count} completed. Checking map status...')
        
        # Wait a bit for SLAM to process new data, then check map again
        timer = self.create_timer(2.0, self._retry_check_map)
        self._retry_timer = timer
    
    def _move_in_direction(self, angle, distance):
        """Move robot in specified direction using omnidirectional cmd_vel"""
        move_duration = distance / self.linear_speed
        
        twist = Twist()
        # Decompose velocity into x and y components
        twist.linear.x = self.linear_speed * math.cos(angle)
        twist.linear.y = self.linear_speed * math.sin(angle)
        
        start_time = self.get_clock().now()
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < move_duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop movement
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f'Moved {distance}m in direction {math.degrees(angle):.1f}°')
    
    def _retry_check_map(self):
        """Helper to retry check_map_and_move (cancels timer first)"""
        if hasattr(self, '_retry_timer'):
            self._retry_timer.cancel()
        self.check_map_and_move()
    
    def move_goal_response_callback(self, future):
        """Handle move goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Safe move goal rejected, will retry after delay')
            timer = self.create_timer(2.0, self._retry_check_map)
            self._retry_timer = timer
            return
        
        self.get_logger().info('Safe move goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_callback)
    
    def move_result_callback(self, future):
        """Handle move completion and check if need to continue"""
        result = future.result().result
        self.get_logger().info(f'Move {self.move_count} completed. Checking map status...')
        
        # Wait a bit for SLAM to process new data, then check map again
        timer = self.create_timer(2.0, self._retry_check_map)
        self._retry_timer = timer


def main(args=None):
    rclpy.init(args=args)
    node = InitialSpinNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    def scan_callback(self, msg):
        """Store latest laser scan data"""
        self.latest_scan = msg
    
    def map_callback(self, msg):
        """Store latest map data"""
        self.latest_map = msg
    
    def is_map_empty(self):
        """
        Check if the map is empty or contains valid obstacle data
        Returns: True if map is empty/invalid, False if map has content
        """
        if self.latest_map is None:
            self.get_logger().info('Map data not received yet')
            return True
        
        map_data = self.latest_map
        
        # Check if map has zero size
        if map_data.info.width == 0 or map_data.info.height == 0:
            self.get_logger().info(f'Map has zero size: {map_data.info.width}x{map_data.info.height}')
            return True
        
        # Check if map data is all unknown (-1) or free (0)
        occupied_cells = sum(1 for cell in map_data.data if cell > 50)  # Occupied threshold
        known_cells = sum(1 for cell in map_data.data if cell >= 0)  # Known cells (not -1)
        total_cells = len(map_data.data)
        
        occupied_ratio = occupied_cells / total_cells if total_cells > 0 else 0
        known_ratio = known_cells / total_cells if total_cells > 0 else 0
        
        self.get_logger().info(
            f'Map status: {map_data.info.width}x{map_data.info.height}, '
            f'occupied: {occupied_cells}/{total_cells} ({occupied_ratio*100:.1f}%), '
            f'known: {known_cells}/{total_cells} ({known_ratio*100:.1f}%)'
        )
        
        # Map is considered valid if it has at least 1% known cells or some obstacles
        is_empty = known_ratio < 0.01 and occupied_cells < 10
        
        if is_empty:
            self.get_logger().warn('Map is still empty or has insufficient data')
        else:
            self.get_logger().info('Map has valid data!')
        
        return is_empty
    
    def find_safest_direction(self):
        """
        Find the safest direction (45-degree cone with no obstacles within min_obstacle_distance)
        Returns: angle in radians relative to robot's current heading
        """
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available, defaulting to forward direction')
            return 0.0
        
        scan = self.latest_scan
        num_points = len(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # Calculate cone size in number of scan points
        cone_half_points = int(self.safe_cone_angle / 2.0 / angle_increment)
        
        best_direction = 0.0
        max_min_distance = 0.0
        
        # Scan through all possible directions
        for center_idx in range(num_points):
            # Get cone range
            start_idx = max(0, center_idx - cone_half_points)
            end_idx = min(num_points - 1, center_idx + cone_half_points)
            
            # Find minimum distance in this cone
            cone_ranges = scan.ranges[start_idx:end_idx+1]
            valid_ranges = [r for r in cone_ranges if scan.range_min < r < scan.range_max]
            
            if not valid_ranges:
                continue
            
            min_distance_in_cone = min(valid_ranges)
            
            # Update best direction if this cone has better clearance
            if min_distance_in_cone > max_min_distance:
                max_min_distance = min_distance_in_cone
                best_direction = angle_min + center_idx * angle_increment
        
        self.get_logger().info(f'Safest direction: {math.degrees(best_direction):.1f}° with {max_min_distance:.2f}m clearance')
        
        # Only move if we have sufficient clearance
        if max_min_distance < self.min_obstacle_distance:
            self.get_logger().warn(f'Insufficient clearance ({max_min_distance:.2f}m < {self.min_obstacle_distance}m), skipping move')
            return None
        
        return best_direction
    
    def execute_initial_spin(self):
        """Execute the initial spin after startup delay"""
        self.timer.cancel()  # One-time execution
        
        self.get_logger().info('Waiting for Spin action server...')
        if not self._spin_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Spin action server not available after 10s. Skipping initial spin.')
            return
        
        self.get_logger().info(f'Executing initial spin of {self.spin_angle} radians (90 degrees)...')
        
        # Create and send spin goal
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = self.spin_angle
        goal_msg.time_allowance.sec = 10
        
        send_goal_future = self._spin_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.spin_goal_response_callback)
    
    def spin_goal_response_callback(self, future):
        """Handle spin goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Initial spin goal rejected')
            return
        
        self.get_logger().info('Initial spin goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)
    
    def spin_result_callback(self, future):
        """Handle spin completion and initiate safe move"""
        result = future.result().result
        self.get_logger().info('Initial spin completed! Now checking map and finding safest direction...')
        self.spin_completed = True
        
        # Wait a bit for scan data to settle
        timer = self.create_timer(1.0, self.check_map_and_move)
        self._check_map_timer = timer
    
    def check_map_and_move(self):
        """Check if map is empty and decide whether to move"""
        # Cancel the timer if it exists
        if hasattr(self, '_check_map_timer'):
            self._check_map_timer.cancel()
        
        if not self.spin_completed:
            return
        
        # Check if map has valid data
        map_is_empty = self.is_map_empty()
        
        if not map_is_empty:
            self.get_logger().info('Map has valid data, initial setup complete!')
            return
        
        # Map is still empty, check move attempts
        if self.move_count >= self.max_move_attempts:
            self.get_logger().warn(f'Reached max move attempts ({self.max_move_attempts}), stopping')
            return
        
        self.get_logger().info(f'Map is empty, initiating move {self.move_count + 1}/{self.max_move_attempts}')
        self.execute_safe_move()
    
    def execute_safe_move(self):
        """Move in the safest direction"""
        safe_angle = self.find_safest_direction()
        
        if safe_angle is None:
            self.get_logger().warn('No safe direction found, will retry after delay')
            timer = self.create_timer(2.0, self._retry_check_map)
            self._retry_timer = timer
            return
        self.get_logger().info('Waiting for NavigateToPose action server...')
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available. Will retry after delay.')
            timer = self.create_timer(2.0, self._retry_check_map)
            self._retry_timer = timer
            returnreate_timer(2.0, self.check_map_and_move, once=True)
            return
        
        # Create goal pose 0.5m in the safest direction
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'base_link'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position: 0.5m in safest direction relative to robot
        goal_msg.pose.pose.position.x = self.move_distance * math.cos(safe_angle)
        goal_msg.pose.pose.position.y = self.move_distance * math.sin(safe_angle)
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation: face the direction of movement
        q = quaternion_from_euler(0, 0, safe_angle)
        goal_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        self.move_count += 1
        self.get_logger().info(f'Moving {self.move_distance}m in direction {math.degrees(safe_angle):.1f}° (attempt {self.move_count})...')
        
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.move_goal_response_callback)
    def _retry_check_map(self):
        """Helper to retry check_map_and_move (cancels timer first)"""
        if hasattr(self, '_retry_timer'):
            self._retry_timer.cancel()
        self.check_map_and_move()
    
    def move_goal_response_callback(self, future):
        """Handle move goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Safe move goal rejected, will retry after delay')
            timer = self.create_timer(2.0, self._retry_check_map)
    def move_result_callback(self, future):
        """Handle move completion and check if need to continue"""
        result = future.result().result
        self.get_logger().info(f'Move {self.move_count} completed. Checking map status...')
        
        # Wait a bit for SLAM to process new data, then check map again
        timer = self.create_timer(2.0, self._retry_check_map)
        self._retry_timer = timer
    
    def move_result_callback(self, future):
        """Handle move completion and check if need to continue"""
        result = future.result().result
        self.get_logger().info(f'Move {self.move_count} completed. Checking map status...')
        
        # Wait a bit for SLAM to process new data, then check map again
        self.create_timer(2.0, self.check_map_and_move, once=True)


def main(args=None):
    rclpy.init(args=args)
    node = InitialSpinNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
