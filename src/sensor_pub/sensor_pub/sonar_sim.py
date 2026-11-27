#!/usr/bin/env python3

import re
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from oculus_ros2.oculus_interfaces.msg import Ping


class SonarSimulator(Node):

    def __init__(self):
        super().__init__('sonar_simulator')

        # 参数配置
        self.declare_parameter('data_dir', '~/Documents/frames_1')
        self.declare_parameter('file_pattern', 'frame_*')
        self.declare_parameter('n_ranges', 401)
        self.declare_parameter('n_beams', 512)
        self.declare_parameter('range', 8.0)
        self.declare_parameter('range_resolution', 0.02)
        self.declare_parameter('frame_id', 'sonar')
        self.declare_parameter('publish_rate', 10.0)

        self.data_dir = Path(
            self.get_parameter('data_dir').value
        ).expanduser()

        self.n_ranges = self.get_parameter('n_ranges').value
        self.n_beams = self.get_parameter('n_beams').value
        self.sonar_range = self.get_parameter('range').value
        self.range_resolution = self.get_parameter(
            'range_resolution'
        ).value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # 获取并排序数据文件
        self.files = list(
            self.data_dir.glob(
                self.get_parameter('file_pattern').value
            )
        )

        def frame_number(path):
            match = re.search(r'frame_(\d+)', path.name)
            return int(match.group(1)) if match else -1

        self.files.sort(key=frame_number)

        if not self.files:
            raise RuntimeError(
                f'No sonar frames found in {self.data_dir}'
            )

        # 检查文件大小
        self.expected_size = self.n_ranges * self.n_beams

        for file in self.files:
            if file.stat().st_size != self.expected_size:
                raise ValueError(
                    f'{file.name}: expected {self.expected_size} bytes, '
                    f'got {file.stat().st_size} bytes'
                )

        self.get_logger().info(
            f'Loaded {len(self.files)} frames, '
            f'image size: {self.n_ranges}x{self.n_beams}'
        )

        # 创建发布者，与 Oculus 节点使用相同的相对话题名
        qos = QoSProfile(depth=1)
        self.publisher = self.create_publisher(
            Ping,
            'ping',
            qos
        )

        self.frame_index = 0

        # 定时发布，默认 10 Hz
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(
            period,
            self.publish_frame
        )

    def publish_frame(self):

        if self.frame_index >= len(self.files):
            self.get_logger().info('All frames published.')
            self.timer.cancel()
            return

        file = self.files[self.frame_index]

        # 读取原始像素数据
        frame = np.fromfile(file, dtype=np.uint8)
        frame = frame.reshape(
            self.n_ranges,
            self.n_beams
        )

        # 创建 Ping 消息
        msg = Ping()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Ping 基本信息
        msg.ping_id = self.frame_index + 1

        # 声呐图像参数
        msg.range = float(self.sonar_range)
        msg.range_resolution = float(self.range_resolution)

        msg.n_ranges = self.n_ranges
        msg.n_beams = self.n_beams

        # 每行字节数和每像素字节数
        msg.sample_size = 1
        msg.step = self.n_beams

        # 填充有效图像数据
        msg.ping_data = frame.flatten().tolist()

        # 发布
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published {file.name} '
            f'({self.frame_index + 1}/{len(self.files)})'
        )

        self.frame_index += 1


def main(args=None):

    rclpy.init(args=args)

    node = SonarSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
