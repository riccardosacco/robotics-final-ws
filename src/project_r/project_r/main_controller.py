#!/usr/bin/env python3
"""
ROS2 Robot Controller for Fire Detection and Navigation

A unified controller that combines basic robot control with fire detection capabilities.
Features:
1. Basic movement control and odometry processing
2. Autonomous navigation using range sensors
3. Fire detection using computer vision
4. State machine for behavior control
5. Celebration dance when fire is found
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
from math import inf
import random
from std_msgs.msg import String, ColorRGBA
import math
import sys
import time
from transforms3d._gohlketransforms import euler_from_quaternion
from typing import Tuple, Dict, List


class RobotState(Enum):
    """Robot states for behavior control."""
    FORWARD = 1     # Move straight until obstacle
    BACKUP = 2      # Back up if too close
    ROTATING = 3    # Rotate to find clear path
    FIRE_FOUND = 4  # Fire detected, stop and report
    CELEBRATING = 5 # Do celebration dance


class RobotController(Node):
    """
    Unified robot controller with fire detection and navigation capabilities.
    
    Combines basic movement control with advanced behaviors for:
    - Autonomous navigation
    - Obstacle avoidance
    - Fire detection
    - Celebration sequences
    """
    
    # Robot control parameters
    UPDATE_RATE = 20  # Hz
    OUT_OF_RANGE = 10.0   # Maximum sensor range [m]
    TARGET_DISTANCE = 0.3  # Desired distance from obstacles [m]
    TOO_CLOSE = 0.2       # Distance that triggers backup [m]
    MIN_FREE_SPACE = 0.5  # Minimum space needed to move forward [m]
    
    # Fire detection thresholds (HSV color space)
    FIRE_LOWER = np.array([15, 100, 100])   # Yellow-orange lower bound
    FIRE_UPPER = np.array([40, 255, 255])   # Yellow-orange upper bound
    FIRE_MIN_AREA = 12000                   # Minimum fire size in pixels

    def __init__(self, node_name: str = "robot_controller"):
        """Initialize the robot controller with all necessary components."""
        super().__init__(node_name)
        
        # Robot state
        self.odom_pose: Pose = None
        self.odom_velocity: Twist = None
        self.current_state = None
        self.next_state = RobotState.FORWARD
        
        # Fire detection state
        self.fire_detected = False
        self.celebration_start_time = None
        self.ignore_fire_detection = False
        
        # Vision processing
        self.cv_bridge = CvBridge()
        
        # Setup ROS2 communication
        self._setup_core_components()
        self._setup_camera()
        self._setup_range_sensors()
        
    def _setup_core_components(self):
        """Set up basic movement control and odometry."""
        # Movement control
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Odometry for position tracking
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            "odom",
            self._on_odometry, 
            10
        )

    def _setup_camera(self):
        """Set up camera and vision processing components."""
        # Camera input
        self.camera_sub = self.create_subscription(
            Image,
            'camera/image_color',
            self._on_camera_image,
            10
        )
        
        # Debug visualization
        self.debug_image_pub = self.create_publisher(Image, 'fire_debug/image', 10)
        self.mask_pub = self.create_publisher(Image, 'fire_debug/mask', 10)

    def _setup_range_sensors(self):
        """Set up range sensors for obstacle detection."""
        # Sensor configuration
        self.sensors: Dict[str, str] = {
            "rear_right": "range_0",
            "front_right": "range_1",
            "rear_left": "range_2",
            "front_left": "range_3",
        }
        
        # Sensor grouping
        self.front_sensors = [self.sensors["front_left"], self.sensors["front_right"]]
        self.rear_sensors = [self.sensors["rear_left"], self.sensors["rear_right"]]
        self.all_sensors = self.front_sensors + self.rear_sensors
        
        # Sensor data storage
        self.range_readings: Dict[str, float] = {}
        
        # Create subscribers
        self.range_subscribers = [
            self.create_subscription(
                Range, 
                sensor, 
                self._create_range_callback(sensor), 
                10
            )
            for sensor in self.all_sensors
        ]

        # Celebration dance publishers
        self.arm_pub = self.create_publisher(Vector3, '/rm0/cmd_arm', 10)
        self.sound_pub = self.create_publisher(String, '/rm0/cmd_sound', 10)
        self.led_color_pub = self.create_publisher(ColorRGBA, '/rm0/leds/color', 10)
        self.led_eff_pub = self.create_publisher(String, '/rm0/leds/effect', 10)

        # Dance step counter
        self.dance_step = 0

    def start(self):
        """Start the control loop."""
        period = 1.0 / self.UPDATE_RATE
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().info("Robot controller started")

    def stop(self):
        """Stop the robot safely."""
        self._send_velocity(0.0, 0.0)
        self.get_logger().info("Robot controller stopped")

    def _send_velocity(self, linear: float, angular: float):
        """Send velocity command to the robot."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_publisher.publish(cmd)

    def _on_odometry(self, msg: Odometry):
        """Process odometry updates."""
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
        # Convert to 2D pose for logging
        x, y, theta = self._get_2d_pose()
        self.get_logger().debug(
            f"Position: x={x:.2f}, y={y:.2f}, θ={theta:.2f}",
            throttle_duration_sec=0.5
        )

    def _get_2d_pose(self) -> Tuple[float, float, float]:
        """Extract 2D pose from 3D odometry."""
        if not self.odom_pose:
            return (0.0, 0.0, 0.0)
            
        q = self.odom_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        return (
            self.odom_pose.position.x,
            self.odom_pose.position.y,
            yaw
        )

    def _create_range_callback(self, sensor: str):
        """Create callback for range sensor updates."""
        def _on_range(msg: Range):
            self.range_readings[sensor] = msg.range if msg.range >= 0.0 else inf
            self.get_logger().debug(
                f"Range sensors: {self.range_readings}",
                throttle_duration_sec=0.5
            )
        return _on_range

    def _on_camera_image(self, msg: Image):
        """Process camera image for fire detection."""
        if self.ignore_fire_detection:
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            fire_detected = self._detect_fire(cv_image)
            
            if fire_detected and not self.fire_detected:
                if self.current_state != RobotState.CELEBRATING:
                    self.get_logger().info("Fire detected!")
                    self.fire_detected = True
                    if self.current_state != RobotState.FIRE_FOUND:
                        self.next_state = RobotState.FIRE_FOUND
            elif not fire_detected:
                self.fire_detected = False
                
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')

    def _detect_fire(self, image) -> bool:
        """Detect fire in image using color segmentation."""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.FIRE_LOWER, self.FIRE_UPPER)
        
        # Clean up mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find fire contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create debug visualization
        debug = image.copy()
        cv2.drawContours(debug, contours, -1, (0,255,0), 2)
        
        # Check for large enough fire areas
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.FIRE_MIN_AREA:
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(debug, (x,y), (x+w,y+h), (0,0,255), 2)
                cv2.putText(debug, f'Fire: {area:.0f}px', (x, y-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                
                # Publish debug images
                self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(debug, "bgr8"))
                self.mask_pub.publish(self.cv_bridge.cv2_to_imgmsg(mask, "mono8"))
                return True
                
        return False

    def _control_loop(self):
        """Main control loop implementing the state machine."""
        # Wait for sensor data
        if not self._check_sensors_ready():
            return

        # Handle state transitions
        if self.next_state != self.current_state:
            self._handle_state_transition()

        # Update current state
        self._update_current_state()

    def _check_sensors_ready(self) -> bool:
        """Check if all sensors are providing data."""
        return (self.odom_pose is not None and 
                len(self.range_readings) == len(self.all_sensors))

    def _handle_state_transition(self):
        """Handle transition between states."""
        self.get_logger().info(f"State: {self.current_state} -> {self.next_state}")
        
        # Initialize new state
        initializers = {
            RobotState.FORWARD: self._init_forward,
            RobotState.BACKUP: self._init_backup,
            RobotState.ROTATING: self._init_rotating,
            RobotState.FIRE_FOUND: self._init_fire_found,
            RobotState.CELEBRATING: self._init_celebrating
        }
        
        if self.next_state in initializers:
            initializers[self.next_state]()
            
        self.current_state = self.next_state

    def _update_current_state(self):
        """Execute current state behavior."""
        updaters = {
            RobotState.FORWARD: self._update_forward,
            RobotState.BACKUP: self._update_backup,
            RobotState.ROTATING: self._update_rotating,
            RobotState.FIRE_FOUND: self._update_fire_found,
            RobotState.CELEBRATING: self._update_celebrating
        }
        
        if self.current_state in updaters:
            updaters[self.current_state]()

    # State initialization methods
    def _init_forward(self):
        """Start moving forward."""
        self.stop()

    def _init_backup(self):
        """Start backing up."""
        self.stop()

    def _init_rotating(self):
        """Start rotating to find path."""
        self.stop()
        self.turn_direction = random.choice([-1, 1])

    def _init_fire_found(self):
        """React to fire detection."""
        self.stop()
        self.get_logger().info("Fire found! Starting celebration!")
        self.ignore_fire_detection = True
        self.celebration_start_time = time.time()
        # Reset dance counter when fire is found
        self.dance_step = 0
        self.next_state = RobotState.CELEBRATING

    def _init_celebrating(self):
        """Start celebration sequence."""
        self.get_logger().info("Starting celebration dance!")

    # State update methods
    def _update_forward(self):
        """Move forward until obstacle detected."""
        if any(self.range_readings[s] < self.TARGET_DISTANCE 
               for s in self.front_sensors):
            self.next_state = RobotState.BACKUP
            return

        self._send_velocity(0.2, 0.0)

    def _update_backup(self):
        """Back up until safe distance reached."""
        if all(self.range_readings[s] > self.TOO_CLOSE 
               for s in self.front_sensors):
            self.next_state = RobotState.ROTATING
            return

        self._send_velocity(-0.1, 0.0)

    def _update_rotating(self):
        """Rotate until clear path found."""
        if all(self.range_readings[s] >= self.MIN_FREE_SPACE 
               for s in self.front_sensors):
            self.next_state = RobotState.FORWARD
            return

        self._send_velocity(0.0, self.turn_direction * 0.5)

    def _update_fire_found(self):
        """Transition to celebration."""
        pass

    def _update_celebrating(self):
        """Perform celebration dance after fire detection."""
        t = self.dance_step
        twist = Twist()
        arm = Vector3()
        led_color = ColorRGBA()
        eff = String()
        # Play sound once at start
        if t == 0:
            sound = String()
            sound.data = 'celebration_tune'
            self.sound_pub.publish(sound)
        # Alternate wheel spins every 5 steps
        twist.angular.z = 1.0 if (t // 5) % 2 == 0 else -1.0
        # Wave arm: up/down on z and left/right on y
        pitch = 0.5 * math.sin((t % 6) * math.pi / 3)
        yaw = 0.5 * math.cos((t % 6) * math.pi / 3)
        arm.x = 0.0
        arm.y = yaw
        arm.z = pitch
        # Flash LEDs red/green every 3 steps
        if (t // 3) % 2 == 0:
            led_color.r, led_color.g, led_color.b = 1.0, 0.0, 0.0
        else:
            led_color.r, led_color.g, led_color.b = 0.0, 1.0, 0.0
        led_color.a = 1.0
        eff.data = 'blink'
        # Publish dance commands
        self.vel_publisher.publish(twist)
        self.arm_pub.publish(arm)
        self.led_color_pub.publish(led_color)
        self.led_eff_pub.publish(eff)
        self.dance_step += 1
        # End dance after 60 steps (~3s at 20Hz)
        if self.dance_step >= 60:
            self.stop()
            self.next_state = None


def main():
    """Launch the robot controller."""
    rclpy.init(args=sys.argv)
    
    controller = RobotController()
    controller.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
