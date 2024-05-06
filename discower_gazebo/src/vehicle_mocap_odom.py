import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped, TwistStamped

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.sub_1 = self.create_subscription(PoseStamped, '/orion/loc/truth/pose', self.pose_cb, 10)
        self.sub_2 = self.create_subscription(TwistStamped, '/orion/loc/truth/twist', self.twist_cb, 10)
        timer_period = 0.001  # seconds
        self.got_pose = False
        self.got_twist = False
        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):

        if self.got_pose and self.got_twist:
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

            msg.q = [self.pose.pose.orientation.w, self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z]
            msg.velocity = [self.twist.twist.linear.x, -self.twist.twist.linear.y, -self.twist.twist.linear.z]
            msg.angular_velocity = [self.twist.twist.angular.x, -self.twist.twist.angular.y, -self.twist.twist.angular.z]
            self.publisher_.publish(msg)

    def pose_cb(self, msg):
        self.pose = msg
        self.got_pose = True

    def twist_cb(self, msg):
        self.twist = msg
        self.got_twist = True

def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()