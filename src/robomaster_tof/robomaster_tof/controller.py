import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys


class ControllerNode(Node):
    def __init__(self, *args, update_step=1 / 60, **kwargs):
        super().__init__(*args, **kwargs)

        # Period of the update timer
        self.update_step = update_step  # [s]

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # NOTE: we're using relative names to specify the topics (i.e., without a
        # leading /). ROS resolves relative names by concatenating them with the
        # namespace in which this node has been started, thus allowing us to
        # specify which RoboMaster should be controlled.

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(self.update_step, self.update_callback)

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.get_logger().debug(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
            throttle_duration_sec=0.5,  # Throttle logging frequency to max 2Hz
        )

    def pose3d_to_2d(self, pose3):
        quaternion = (pose3.orientation.x, pose3.orientation.y, pose3.orientation.z, pose3.orientation.w)

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        pose2 = (pose3.position.x, pose3.position.y, yaw)  # x position  # y position  # theta orientation

        return pose2

    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # [m/s]
        cmd_vel.angular.z = 0.0  # [rad/s]

        # Publish the command
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerNode()
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
