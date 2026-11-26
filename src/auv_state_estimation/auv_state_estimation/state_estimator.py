
import rclpy
from rclpy.node import Node

import math
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def rotate_vector(q, v):
    # 将机体坐标系向量转换到世界坐标系
    qv = np.array([q[0], q[1], q[2]])
    w = q[3]

    return (
        v
        + 2 * w * np.cross(qv, v)
        + 2 * np.cross(qv, np.cross(qv, v))
    )


class AUVStateEstimator(Node):

    def __init__(self):
        super().__init__('auv_state_estimator')

        # AUV 当前状态
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

        # 四元数顺序：x, y, z, w
        self.orientation = np.array([0., 0., 0., 1.])

        self.last_imu_time = None
        self.last_dvl_time = None

        # 订阅传感器
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.dvl_sub = self.create_subscription(
            TwistStamped,
            '/dvl/velocity',
            self.dvl_callback,
            10
        )

        # 发布位姿
        self.odom_pub = self.create_publisher(
            Odometry,
            '/auv/odometry',
            10
        )

        # 发布动态坐标变换
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('AUV State Estimator Started')

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def imu_callback(self, msg):

        t = self.stamp_to_sec(msg.header.stamp)

        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t

        if dt <= 0 or dt > 0.5:
            return

        # 读取陀螺仪角速度
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        omega = np.array([wx, wy, wz, 0.0])

        # 四元数积分
        q_dot = 0.5 * quaternion_multiply(
            self.orientation, omega
        )

        self.orientation += q_dot * dt

        # 四元数归一化
        norm = np.linalg.norm(self.orientation)

        if norm > 1e-12:
            self.orientation /= norm

    def dvl_callback(self, msg):

        t = self.stamp_to_sec(msg.header.stamp)

        if self.last_dvl_time is None:
            self.last_dvl_time = t
            return

        dt = t - self.last_dvl_time
        self.last_dvl_time = t

        if dt <= 0 or dt > 0.5:
            return

        # DVL 机体坐标系速度
        self.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])

        # 转换到世界坐标系
        velocity_world = rotate_vector(
            self.orientation,
            self.velocity
        )

        # 更新位置
        self.position += velocity_world * dt

        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):

        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 位置
        odom.pose.pose.position.x = float(self.position[0])
        odom.pose.pose.position.y = float(self.position[1])
        odom.pose.pose.position.z = float(self.position[2])

        # 姿态
        q = self.orientation

        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        # 机体坐标系速度
        odom.twist.twist.linear.x = float(self.velocity[0])
        odom.twist.twist.linear.y = float(self.velocity[1])
        odom.twist.twist.linear.z = float(self.velocity[2])

        self.odom_pub.publish(odom)

        # 发布 TF
        tf = TransformStamped()

        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = float(self.position[0])
        tf.transform.translation.y = float(self.position[1])
        tf.transform.translation.z = float(self.position[2])

        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):

    rclpy.init(args=args)

    node = AUVStateEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
