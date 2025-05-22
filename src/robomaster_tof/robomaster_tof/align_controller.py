import rclpy
from rclpy.node import Node

from robomaster_tof.controller import ControllerNode
from robomaster_tof.pid import PID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from copy import deepcopy
from enum import Enum
from math import sin, cos, inf
import sys


class RMState(Enum):
    # Initially, move straight towards the wall
    FORWARD = 1
    # Once the wall is in range of the front sensors, blidly turn until it becomes in range of the rear sensors
    TURNING = 2
    # Use the rear sensors to align the RoboMaster so that it is perpendicular to the wall
    ALIGNING = 3
    # Move away from the wall until the RoboMaster is 2m outturt
    BACKWARD = 4
    # Stop and stand still
    DONE = 5


class AlignController(ControllerNode):
    # Period of the update timer, set based on update frequencies of proximity sensors (10Hz) and odom (20Hz)
    UPDATE_STEP = 1 / 20

    # Max range of the RoboMaster's proximity sensors
    OUT_OF_RANGE = 10.0

    # Target distance of the robot from the wall at the end of FORWARD
    TARGET_DISTANCE = 0.2

    # Target difference between the distance measured by the two distance sensors at the end of ALIGNING and
    # number of time steps it needs to be maintained to be considered aligned
    TARGET_ANGLE_ERROR = 0.02
    TARGET_ANGLE_COUNT = 6

    # Target distance of the robot from the wall at the end of BACKWARD
    TARGET_DISTANCE_ERROR = 0.01

    def __init__(self):
        super().__init__("align_controller", update_step=self.UPDATE_STEP)

        # Initialize the state machine
        self.current_state = None
        self.next_state = RMState.FORWARD
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

        # Create PID controllers for the ALIGNING and BACKWARD states
        self.aligning_controller = PID(2, 0, 0.5, max_out=0.5, min_out=-0.5)
        self.backward_controller = PID(3, 0, 0.3, max_out=0.3)

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

            if self.next_state == RMState.TURNING:
                self.init_turning()
            elif self.next_state == RMState.ALIGNING:
                self.init_aligning()
            elif self.next_state == RMState.BACKWARD:
                self.init_backward()
            elif self.next_state == RMState.DONE:
                self.init_done()

            self.current_state = self.next_state

        # Call update code for the current state
        if self.current_state == RMState.FORWARD:
            self.update_forward()
        elif self.current_state == RMState.TURNING:
            self.update_turning()
        elif self.current_state == RMState.ALIGNING:
            self.update_aligning()
        elif self.current_state == RMState.BACKWARD:
            self.update_backward()
        elif self.current_state == RMState.DONE:
            return

    def update_forward(self):
        # Check if the robot reached the wall. Waiting until two sensors see the wall ensures that one of
        # them is the center_left or center_right one, allowing us to detect in which direction we should turn.
        if any([self.range_sensors_readings[sensor] < self.TARGET_DISTANCE for sensor in self.front_sensors]):
            self.next_state = RMState.TURNING
            return

        # Just move forward with constant velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3  # [m/s]
        cmd_vel.angular.z = 0.0  # [rad/s]
        self.vel_publisher.publish(cmd_vel)

    def init_turning(self):
        self.stop()

        # Use the difference between the distances measured by the two proximity sensors to detect whether the robot
        # is facing the wall and decide in which direction turn the robot
        delta = (
            self.range_sensors_readings[self.sensor_names["front_left"]]
            - self.range_sensors_readings[self.sensor_names["front_right"]]
        )

        self.turn_direction = 1  # counter-clockwise
        if delta < 0:
            self.turn_direction = -1  # clockwise

    def update_turning(self):
        # As soon as one of the two rear proximity sensors detects the wall, stop turning blindly and start aligning
        if any([self.range_sensors_readings[sensor] < self.OUT_OF_RANGE for sensor in self.rear_sensors]):
            self.next_state = RMState.ALIGNING
            return

        # Just rotate in place with constant velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # [m/s]
        cmd_vel.angular.z = self.turn_direction * 0.3  # [rad/s]
        self.vel_publisher.publish(cmd_vel)

    def init_aligning(self):
        self.stop()

        # Smooth measurement noise while aligning with the wall
        self.aligned_counter = 0

    def update_aligning(self):
        # Use the difference between the distances measured by the two proximity sensors to detect whether the robot
        # is aligned with the wall
        target_delta = 0
        delta = (
            self.range_sensors_readings[self.sensor_names["rear_left"]]
            - self.range_sensors_readings[self.sensor_names["rear_right"]]
        )

        # Compute error and clip it to the range of the proximity sensors (deals with out-of-range inf values)
        error = delta - target_delta
        error = max(-self.OUT_OF_RANGE, min(error, self.OUT_OF_RANGE))

        # Ensure that the error stays below target for a few cycles to smooth out measurement noise
        if abs(error) <= self.TARGET_ANGLE_ERROR:
            self.get_logger().info(f"state_machine: aligned {error}")
            self.aligned_counter += 1
        else:
            self.aligned_counter = 0

        if self.aligned_counter == self.TARGET_ANGLE_COUNT:
            self.next_state = RMState.BACKWARD
            return

        # Use the PID controller to minimize the difference between the two proximity sensors
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # [m/s]
        cmd_vel.angular.z = self.aligning_controller.step(error, self.update_step)  # [rad/s]

        self.get_logger().info("cmd_vel: " + str(cmd_vel))
        # Publish the command
        self.vel_publisher.publish(cmd_vel)

    def init_backward(self):
        self.stop()

        # Move that robot in such a way its reference frame is as close as possible to a point that is 2 meters
        # from the wall, in this case relying on odometry, and then stop it

        # The target pose is 2m from the wall, perpedicularly:
        #  * odom_pose represents the current position estimated by the odometry, in world frame
        #  * At this point the current position of the RoboMaster is perpendicular to the wall, at an unspecified distance
        #  * Measure this distance and combine it with odom_pose to compute the coordinates of the target pose in world frame
        wall_distance = self.range_sensors_readings[self.sensor_names["rear_left"]]
        target_distance = 2.0 - wall_distance
        _, _, yaw = self.pose3d_to_2d(self.odom_pose)

        self.target_pose = deepcopy(self.odom_pose)
        self.target_pose.position.x += target_distance * cos(yaw)
        self.target_pose.position.y += target_distance * sin(yaw)

        self.get_logger().info(f"align_controller: backward {self.target_pose} from {self.odom_pose}")

    def update_backward(self):
        # Linear distance from target pose, positive if the robot needs to go forward, negative if backwards. This is needed
        # because the RoboMaster will often overshoot its target before converging
        error = self.signed_distance(self.target_pose, self.odom_pose)

        if error <= self.TARGET_DISTANCE_ERROR:
            self.next_state = RMState.DONE
            return

        # Use the PID controller to minimize the distance from the target pose
        cmd_vel = Twist()
        cmd_vel.linear.x = self.backward_controller.step(error, self.update_step)  # [m/s]
        cmd_vel.angular.z = 0.0  # [rad/s]
        self.vel_publisher.publish(cmd_vel)

    def init_done(self):
        self.stop()

    def signed_distance(self, goal_pose, current_pose):
        """Signed distance between current pose and the goal pose, along the current theta of the robot"""
        current_x, current_y, current_yaw = self.pose3d_to_2d(current_pose)
        goal_x, goal_y, _ = self.pose3d_to_2d(goal_pose)

        a = current_x * cos(current_yaw) + current_y * sin(current_yaw)
        b = goal_x * cos(current_yaw) + goal_y * sin(current_yaw)

        return b - a


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = AlignController()
    node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the RoboMaster is stopped before exiting
    node.stop()


if __name__ == "__main__":
    main()
