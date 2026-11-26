
import csv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped


class CSVSensorPublisher(Node):

    def __init__(self):
        super().__init__('csv_sensor_publisher')

        # ROS 参数
        self.declare_parameter('csv_path', 'sensor_data.csv')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('frame_id', 'base_link')

        csv_path = self.get_parameter('csv_path').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        # 读取 CSV
        with open(csv_path, 'r', newline='', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            self.data = list(reader)

        if not self.data:
            raise ValueError('CSV 文件为空！')

        # 检查必要字段
        required_columns = [
            'IMU_GX', 'IMU_GY', 'IMU_GZ',
            'IMU_AX', 'IMU_AY', 'IMU_AZ',
            'DVL_BI_tv', 'DVL_BI_lv', 'DVL_BI_nv'
        ]

        missing = set(required_columns) - set(self.data[0].keys())
        if missing:
            raise ValueError(f'CSV 缺少字段: {missing}')

        # 创建发布者
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data_raw', 10
        )

        self.dvl_pub = self.create_publisher(
            TwistStamped, '/dvl/twist', 10
        )

        # 数据索引
        self.index = 0

        # 定时器：默认每 0.1 秒执行一次
        self.timer = self.create_timer(
            1.0 / self.rate_hz,
            self.publish_data
        )

        self.get_logger().info(
            f'Loaded {len(self.data)} rows from {csv_path}'
        )

    def publish_data(self):

        # CSV 数据读取完毕
        if self.index >= len(self.data):
            self.timer.cancel()
            self.get_logger().info('CSV playback finished!')
            return

        row = self.data[self.index]

        # 使用同一个 ROS 时间戳
        stamp = self.get_clock().now().to_msg()

        # ---------------- IMU ----------------
        imu_msg = Imu()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id

        # 三轴角速度
        imu_msg.angular_velocity.x = float(row['IMU_GX'])
        imu_msg.angular_velocity.y = float(row['IMU_GY'])
        imu_msg.angular_velocity.z = float(row['IMU_GZ'])

        # 三轴线加速度
        imu_msg.linear_acceleration.x = float(row['IMU_AX'])
        imu_msg.linear_acceleration.y = float(row['IMU_AY'])
        imu_msg.linear_acceleration.z = float(row['IMU_AZ'])

        # 暂无姿态数据
        imu_msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        # ---------------- DVL ----------------
        dvl_msg = TwistStamped()

        dvl_msg.header.stamp = stamp
        dvl_msg.header.frame_id = self.frame_id

        # 三轴速度
        dvl_msg.twist.linear.x = float(row['DVL_BI_tv'])
        dvl_msg.twist.linear.y = float(row['DVL_BI_lv'])
        dvl_msg.twist.linear.z = float(row['DVL_BI_nv'])

        self.dvl_pub.publish(dvl_msg)

        self.index += 1


def main(args=None):

    rclpy.init(args=args)

    node = CSVSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
