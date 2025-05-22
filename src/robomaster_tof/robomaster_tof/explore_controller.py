import rclpy
from rclpy.node import Node

from robomaster_tof.controller import ControllerNode
from robomaster_tof.pid import PID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from copy import deepcopy
from enum import Enum
from math import sin, cos, inf
import random
import sys


class RMState(Enum):
    # Initially, move straight until the robot reaches an obstacle
    FORWARD = 1
    # Check if the robot didn't stop fast enough and hit the obstacle
    BACKUP = 2
    #  Rotate in a random direction until the robot is clear from obstacles
    ROTATING = 3


class ExploreController(ControllerNode):
    # Period of the update timer, set based on update frequencies of proximity sensors (10Hz) and odom (20Hz)
    UPDATE_STEP = 1 / 20

    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 10.0

    # Target distance of the robot from the wall at the end of FORWARD
    TARGET_DISTANCE = 0.3

    # Minimum distance from the wall to be able to rotate in place
    TOO_CLOSE = 0.2

    # Minimum distance from the wall to be able to move forward
    MIN_FREE_SPACE = 0.5

    # Target difference between the distance measured by the two distance sensors
    TARGET_ERROR = 0.01

    def __init__(self):
        super().__init__("explore_controller", update_step=self.UPDATE_STEP)

        # Initialize the state machine
        self.current_state = None
        self.next_state = RMState.FORWARD

        # Subscribe to all proximity sensors at the same time
        self.sensor_names = {
            "rear_right": "range_0",
            "front_right": "range_1",
            "rear_left": "range_2",
            "front_left": "range_3",
        }
        # Subscribe to all proximity sensors at the same time
        self.front_sensors = [self.sensor_names["front_left"], self.sensor_names["front_right"]]
        self.rear_sensors = [self.sensor_names["rear_left"], self.sensor_names["rear_right"]]
        self.range_sensors = self.front_sensors + self.rear_sensors
        self.range_sensors_readings = dict()
        self.range_sensors_subscribers = [
            self.create_subscription(Range, f"{sensor}", self.create_range_sensors_callback(sensor), 10)
            for sensor in self.range_sensors
        ]

    def create_range_sensors_callback(self, sensor):
        # Create a callback function that has access to both the message and the name of the sensor that sent it
        def proximity_callback(msg):
            self.range_sensors_readings[sensor] = msg.range if msg.range >= 0.0 else inf

            self.get_logger().debug(
                f"proximity: {self.range_sensors_readings}",
                throttle_duration_sec=0.5,  # Throttle logging frequency to max 2Hz
            )

        return proximity_callback

    def update_callback(self):
        # Wait until the first update is received from odometry and each proximity sensor
        if self.odom_pose is None or len(self.range_sensors_readings) < len(self.range_sensors):
            return

        # Check whether the state machine was asked to transition to a new
        # state in the previous timestep. In that case, call the initialization
        # code for the new state.
        if self.next_state != self.current_state:
            self.get_logger().info(f"state_machine: transitioning from {self.current_state} to {self.next_state}")

            if self.next_state == RMState.FORWARD:
                self.init_forward()
            elif self.next_state == RMState.BACKUP:
                self.init_backup()
            elif self.next_state == RMState.ROTATING:
                self.init_rotating()

            self.current_state = self.next_state

        # Call update code for the current state
        if self.current_state == RMState.FORWARD:
            self.update_forward()
        elif self.current_state == RMState.BACKUP:
            self.update_backup()
        elif self.current_state == RMState.ROTATING:
            self.update_rotating()

    def init_forward(self):
        self.stop()

    def update_forward(self):
        # Check if the robot reached an obstacle it cannot pass through.
        if any(self.range_sensors_readings[sensor] < self.TARGET_DISTANCE for sensor in self.front_sensors):
            self.next_state = RMState.BACKUP
            return

        # Just move forward with constant velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # [m/s]
        cmd_vel.angular.z = 0.0  # [rad/s]
        self.vel_publisher.publish(cmd_vel)

    def init_backup(self):
        self.stop()

    def update_backup(self):
        # Check if the robot didn't stop fast enough and hit the wall
        if all(self.range_sensors_readings[sensor] > self.TOO_CLOSE for sensor in self.front_sensors):
            self.next_state = RMState.ROTATING
            return

        # Slowly back up to clear the obstacle
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.1  # [m/s]
        cmd_vel.angular.z = 0.0  # [rad/s]
        self.vel_publisher.publish(cmd_vel)

    def init_rotating(self):
        self.stop()

        # Choose a random rotation direction to clear the obstacle
        self.turn_direction = random.sample([-1, 1], 1)[0]

    def update_rotating(self):
        if all(self.range_sensors_readings[sensor] >= self.MIN_FREE_SPACE for sensor in self.front_sensors):
            self.next_state = RMState.FORWARD
            return

        # Just rotate in place with constant velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # [m/s]
        cmd_vel.angular.z = self.turn_direction * 0.5  # [rad/s]
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ExploreController()
    node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == "__main__":
    main()
