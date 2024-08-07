import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np

from pyquaternion import Quaternion

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.running_simulation = self.get_parameter_or('simulated', False)
        self.publisher_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)
        if self.running_simulation:
            self.sub_1 = self.create_subscription(PoseStamped, '/orion/loc/truth/pose', self.pose_cb, 1)
            self.sub_2 = self.create_subscription(TwistStamped, '/orion/loc/truth/twist', self.twist_cb, 1)
        else:
            self.sub_1 = self.create_subscription(Odometry, '/snap/odom', self.odom_cb, 1)

        timer_period = 0.01  # seconds
        self.got_pose = False
        self.got_twist = False
        self.got_odom = False
        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.odom = Odometry()
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):

        if (self.got_pose and self.got_twist) or (self.got_odom):
            msg = VehicleOdometry()

            # Set time
            time_s, time_ns = self.get_clock().now().seconds_nanoseconds()
            time_us = (time_s * 1000000) + (time_ns / 1000)
            msg.timestamp = int(time_us)

            # Set frames
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.velocity_frame = VehicleOdometry.POSE_FRAME_FRD

            # NED pose
            msg.position = [self.pose.pose.position.y, self.pose.pose.position.x, -self.pose.pose.position.z]

            att_q = Quaternion(self.pose.pose.orientation.w, self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z)
            msg.q = self.rotateQuaternion(Quaternion(att_q))
            msg.velocity = [self.twist.twist.linear.x, -self.twist.twist.linear.y, -self.twist.twist.linear.z]
            msg.angular_velocity = [self.twist.twist.angular.x, -self.twist.twist.angular.y, -self.twist.twist.angular.z]
            self.publisher_.publish(msg)

    def rotateQuaternion(self, q_FLU_to_ENU):
        # quats are w x y z
        q_FLU_to_FRD = Quaternion(0, 1, 0, 0)
        q_ENU_to_NED = Quaternion(0, 0.70711, 0.70711, 0)

        q_rot = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.inverse
        return [q_rot.w, q_rot.x, q_rot.y, q_rot.z]

    def pose_cb(self, msg):
        self.pose = msg
        self.got_pose = True

    def twist_cb(self, msg):
        self.twist = msg
        self.got_twist = True

    def odom_cb(self, msg):
        self.pose = msg.pose
        self.twist = msg.twist
        self.got_odom = True


def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()