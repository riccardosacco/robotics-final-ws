import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Range, Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import cv2
import numpy as np

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer_node')
        
        # Initialize ROS publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to all four ToF sensors
        self.ranges = [None] * 4
        for i in range(4):
            self.create_subscription(Range, f'range_{i}', 
                                  lambda msg, idx=i: self.range_callback(msg, idx), 10)
                                  
        # Initialize CV bridge and camera subscription
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_color', self.image_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Robot state
        self.pose = Point()
        self.yaw = 0.0
        self.marker_found = False
        self.last_escape_direction = None
        self.escape_start_time = None
        self.stuck_detection_count = 0
        self.last_position = Point()
        self.position_history = []
        self.reverse_time = None
        self.MAX_REVERSE_TIME = 1.0  # Maximum time to reverse in seconds
        
        # Vision parameters
        self.red_lower = np.array([0, 100, 100])
        self.red_upper = np.array([10, 255, 255])
        self.min_marker_area = 500  # minimum area in pixels to consider as marker
        
        # Constants
        self.MIN_OBSTACLE_DIST = 0.4  # meters
        self.DANGER_OBSTACLE_DIST = 0.25  # meters
        self.ANGULAR_SPEED = 1.2  # rad/s (increased for faster turns)
        self.LINEAR_SPEED = 0.2  # m/s
        self.MIN_ESCAPE_DURATION = 1.5  # seconds
        self.STUCK_THRESHOLD = 0.05  # meters
        self.HISTORY_SIZE = 10  # number of positions to keep
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Explorer node initialized with improved navigation')

    def range_callback(self, msg, sensor_idx):
        """Process ToF sensor data"""
        self.ranges[sensor_idx] = msg

    def image_callback(self, msg):
        """Process camera image for red marker detection"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for red color
            mask = cv2.inRange(hsv, self.red_lower, self.red_upper)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Check each contour
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_marker_area:
                    self.marker_found = True
                    self.get_logger().info(f'Red marker found! Area: {area:.2f} pixels')
                    self.celebrate()
                    return
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def odom_callback(self, msg):
        """Process odometry data"""
        self.pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = quat2euler([
            orientation.w, orientation.x, orientation.y, orientation.z
        ])

    def check_obstacles(self):
        """Check for obstacles using front ToF sensors only (0 and 3)"""
        if self.ranges[0] is None or self.ranges[3] is None:
            return None

        # Get readings from front sensors
        front_ranges = [self.ranges[0].range, self.ranges[3].range]
        min_distance = float('inf')
        
        for range_val in front_ranges:
            # Skip invalid readings
            if range_val < self.ranges[0].min_range:
                continue
                
            # Treat maximum range readings as safe distance
            if range_val >= self.ranges[0].max_range:
                continue
                
            if range_val < min_distance:
                min_distance = range_val

        if min_distance == float('inf'):
            return None

        return min_distance

    def is_stuck(self):
        """Detect if robot is stuck by checking recent positions"""
        current_pos = self.pose
        
        # Add current position to history
        self.position_history.append((current_pos.x, current_pos.y))
        if len(self.position_history) > self.HISTORY_SIZE:
            self.position_history.pop(0)
            
        # Need enough history to check
        if len(self.position_history) < self.HISTORY_SIZE:
            return False
            
        # Calculate maximum distance from oldest position
        start_x, start_y = self.position_history[0]
        max_distance = 0
        for x, y in self.position_history[1:]:
            distance = math.sqrt((x - start_x)**2 + (y - start_y)**2)
            max_distance = max(max_distance, distance)
            
        return max_distance < self.STUCK_THRESHOLD

    def is_reversing_too_long(self):
        """Check if the robot has been reversing for too long"""
        if self.reverse_time is None:
            return False
        current_time = self.get_clock().now()
        time_reversing = (current_time - self.reverse_time).nanoseconds / 1e9
        return time_reversing > self.MAX_REVERSE_TIME

    def find_escape_direction(self):
        """Find safe direction to turn using front sensors"""
        if self.ranges[0] is None or self.ranges[3] is None:
            return 1  # Default right turn if no readings
            
        # Check if we should maintain current escape direction
        current_time = self.get_clock().now()
        if (self.last_escape_direction is not None and 
            self.escape_start_time is not None):
            time_diff = (current_time - self.escape_start_time).nanoseconds / 1e9
            if time_diff < self.MIN_ESCAPE_DURATION:
                return self.last_escape_direction
        
        # Get readings from front sensors
        front_center = self.ranges[0].range
        front_right = self.ranges[3].range
        
        # If either sensor is at max range, prefer that direction
        if front_center >= self.ranges[0].max_range:
            new_direction = -1  # Turn left if front is clear
        elif front_right >= self.ranges[3].max_range:
            new_direction = 1   # Turn right if right is clear
        else:
            # Turn toward the direction with more space
            new_direction = 1 if front_right > front_center else -1
        
        # Update escape state
        self.last_escape_direction = new_direction
        self.escape_start_time = current_time
        
        return new_direction

    def random_walk(self):
        """Improved obstacle avoidance using front sensors with better recovery"""
        cmd = Twist()
        
        # Check for obstacles
        min_distance = self.check_obstacles()
        
        if min_distance is None:
            # No valid readings, make a random turn
            cmd.angular.z = self.ANGULAR_SPEED * (1 if random.random() > 0.5 else -1)
            self.cmd_vel_pub.publish(cmd)
            return
            
        # Check if stuck
        if self.is_stuck():
            self.stuck_detection_count += 1
            if self.stuck_detection_count > 5:  # If stuck for multiple cycles
                self.get_logger().info('Stuck detected! Executing recovery...')
                
                # If we've been reversing too long, switch to rotation
                if self.is_reversing_too_long():
                    self.get_logger().info('Reversing timeout - switching to rotation')
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.ANGULAR_SPEED * 1.5 * (1 if random.random() > 0.5 else -1)
                    self.reverse_time = None
                else:
                    # Start/continue reversing
                    if self.reverse_time is None:
                        self.reverse_time = self.get_clock().now()
                    cmd.linear.x = -0.15
                    cmd.angular.z = self.ANGULAR_SPEED * 0.8 * (1 if random.random() > 0.5 else -1)
                
                self.cmd_vel_pub.publish(cmd)
                return
        else:
            self.stuck_detection_count = 0
            self.reverse_time = None
            
        if min_distance < self.DANGER_OBSTACLE_DIST:
            # Emergency - turn in place
            turn_direction = self.find_escape_direction()
            cmd.linear.x = 0.0  # Stop forward motion
            cmd.angular.z = self.ANGULAR_SPEED * turn_direction * 1.2  # Faster turn
            
        elif min_distance < self.MIN_OBSTACLE_DIST:
            # Normal avoidance - turn while moving slowly
            turn_direction = self.find_escape_direction()
            cmd.linear.x = self.LINEAR_SPEED * 0.2  # Very slow forward movement
            cmd.angular.z = self.ANGULAR_SPEED * turn_direction
            
        else:
            # Clear path - move forward with slight random rotation
            cmd.linear.x = self.LINEAR_SPEED
            if random.random() < 0.05:  # 5% chance to add small turn
                cmd.angular.z = (random.random() - 0.5) * self.ANGULAR_SPEED * 0.2
                
            # Reset escape direction and reverse time when clear
            self.last_escape_direction = None
            self.escape_start_time = None
            self.reverse_time = None
        
        self.cmd_vel_pub.publish(cmd)

    def celebrate(self):
        """Perform a celebration dance when marker is found"""
        # Stop any current motion
        self.cmd_vel_pub.publish(Twist())
        
        # Create a dance sequence
        dance_moves = [
            (0.0, 1.0, 2.0),   # Spin right
            (0.0, -1.0, 2.0),  # Spin left
            (0.2, 0.0, 1.0),   # Move forward
            (-0.2, 0.0, 1.0),  # Move backward
        ]
        
        for linear_x, angular_z, duration in dance_moves:
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            
            # Publish the move command
            self.cmd_vel_pub.publish(cmd)
            
            # Sleep for the duration
            rclpy.spin_once(self, timeout_sec=duration)
        
        # Stop after dance
        self.cmd_vel_pub.publish(Twist())
        
    def control_loop(self):
        """Main control loop"""
        if not self.marker_found:
            self.random_walk()


def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
