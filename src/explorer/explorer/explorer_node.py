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
        self.MIN_ESCAPE_DURATION = 1.0  # minimum seconds to keep same escape direction
        
        # Vision parameters
        self.red_lower = np.array([0, 100, 100])
        self.red_upper = np.array([10, 255, 255])
        self.min_marker_area = 500  # minimum area in pixels to consider as marker
        
        # Constants
        self.MIN_OBSTACLE_DIST = 0.6  # meters
        self.DANGER_OBSTACLE_DIST = 0.4  # meters
        self.ANGULAR_SPEED = 0.5  # rad/s
        self.LINEAR_SPEED = 0.2  # m/s
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Explorer node initialized with camera and ToF sensors')

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
        """Check for obstacles and edges using ToF sensors"""
        if None in self.ranges:
            return None

        # Get valid readings from all sensors
        min_distance = float('inf')
        for range_msg in self.ranges:
            # Skip invalid readings
            if range_msg.range < range_msg.min_range:
                continue
                
            # Treat maximum range readings as close obstacles (potential edges)
            if range_msg.range >= range_msg.max_range:
                return self.DANGER_OBSTACLE_DIST
                
            if range_msg.range < min_distance:
                min_distance = range_msg.range

        if min_distance == float('inf'):
            return None

        return min_distance

    def find_escape_direction(self):
        """Find safe direction to turn"""
        if None in self.ranges:
            return 1  # Default right turn if no readings
            
        # Get the two front sensors (assumed to be 0 and 3)
        left_dist = self.ranges[0].range
        right_dist = self.ranges[3].range
        
        # If either reading is at max range (edge detected), turn away from it
        if left_dist >= self.ranges[0].max_range:
            return 1  # Turn right
        if right_dist >= self.ranges[3].max_range:
            return -1  # Turn left
            
        # Otherwise turn toward more open space
        return 1 if right_dist > left_dist else -1

    def random_walk(self):
        """Simple obstacle avoidance with edge detection"""
        cmd = Twist()
        
        # Check for obstacles or edges
        min_distance = self.check_obstacles()
        
        if min_distance is None:
            # No valid readings, stop for safety
            self.cmd_vel_pub.publish(Twist())
            return
            
        if min_distance < self.DANGER_OBSTACLE_DIST:
            # Emergency - stop and turn
            turn_direction = self.find_escape_direction()
            cmd.linear.x = 0.0
            cmd.angular.z = self.ANGULAR_SPEED * turn_direction
            
        elif min_distance < self.MIN_OBSTACLE_DIST:
            # Normal avoidance - slow down and turn
            turn_direction = self.find_escape_direction()
            cmd.linear.x = self.LINEAR_SPEED * 0.5  # Half speed
            cmd.angular.z = self.ANGULAR_SPEED * turn_direction * 0.8
            
        else:
            # Clear path - move forward with slight random rotation
            cmd.linear.x = self.LINEAR_SPEED
            if random.random() < 0.1:  # 10% chance to add small turn
                cmd.angular.z = (random.random() - 0.5) * self.ANGULAR_SPEED * 0.3
        
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
