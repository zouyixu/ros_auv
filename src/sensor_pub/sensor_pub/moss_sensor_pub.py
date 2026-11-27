import socket
import struct
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped


# ============================================================
# 1. MOOS V10 通信协议
# ============================================================

MOOS_PROTOCOL = b"ELKS CAN'T DANCE 2/8/10".ljust(32, b"\0")
MAX_PACKET_SIZE = 64 * 1024 * 1024

IMU_VARIABLES = [
    "IMU_GX", "IMU_GY", "IMU_GZ",
    "IMU_AX", "IMU_AY", "IMU_AZ",
    "IMU_ROLL", "IMU_PITCH", "IMU_HEADING",
]

DVL_VARIABLES = [
    "DVL_BI_tv",
    "DVL_BI_lv",
    "DVL_BI_nv",
    "DVL_BD_altitude",
]


def pack_string(value):
    data = value.encode("utf-8")
    return struct.pack("<i", len(data)) + data


def pack_message(
    msg_type, key, value=0.0,
    source="", text="", data_type="D"
):
    body = b"".join([
        struct.pack("<i", 0),
        msg_type.encode("ascii"),
        data_type.encode("ascii"),
        pack_string(source),
        pack_string(""),
        pack_string(""),
        pack_string(key),
        struct.pack("<ddd", time.time(), value, -1.0),
        pack_string(text),
    ])

    return struct.pack("<i", len(body) + 4) + body


def pack_packet(messages):
    payload = b"".join(messages)
    return struct.pack(
        "<iiB", len(payload) + 9, len(messages), 0
    ) + payload


def read_exact(sock, size):
    chunks = bytearray()

    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))

        if not chunk:
            raise ConnectionError("MOOSDB disconnected")

        chunks.extend(chunk)

    return bytes(chunks)


def read_string(data, offset):
    size = struct.unpack_from("<i", data, offset)[0]
    offset += 4

    if size < 0 or offset + size > len(data):
        raise ValueError("Invalid MOOS string length")

    text = data[offset:offset + size].decode(
        "utf-8", errors="replace"
    )

    return text, offset + size


def read_packet(sock):
    header = read_exact(sock, 4)
    packet_size = struct.unpack("<i", header)[0]

    if not 9 <= packet_size <= MAX_PACKET_SIZE:
        raise ValueError("Invalid MOOS packet size")

    packet = header + read_exact(sock, packet_size - 4)

    message_count, compressed = struct.unpack_from(
        "<iB", packet, 4
    )

    if compressed:
        raise ValueError("Compressed packets unsupported")

    if message_count < 0:
        raise ValueError("Invalid message count")

    messages = []
    offset = 9

    for _ in range(message_count):
        message_size = struct.unpack_from(
            "<i", packet, offset
        )[0]

        end = offset + message_size

        if message_size < 54 or end > len(packet):
            raise ValueError("Invalid MOOS message size")

        msg_type = chr(packet[offset + 8])
        data_type = chr(packet[offset + 9])

        cursor = offset + 10

        for _ in range(3):
            _, cursor = read_string(packet, cursor)

        name, cursor = read_string(packet, cursor)

        timestamp, value, _ = struct.unpack_from(
            "<ddd", packet, cursor
        )

        text, cursor = read_string(packet, cursor + 24)

        if cursor != end:
            raise ValueError("Invalid MOOS message content")

        messages.append({
            "type": msg_type,
            "data_type": data_type,
            "name": name,
            "timestamp": timestamp,
            "value": value,
            "text": text,
        })

        offset = end

    return messages


def message_value(message):
    if message["data_type"] == "D":
        return float(message["value"])

    text = message["text"]

    try:
        return float(text)
    except (TypeError, ValueError):
        return text


# ============================================================
# 2. MOOS 客户端
# ============================================================

class MoosSensorClient:

    def __init__(self, host, port, client_name):

        self.host = host
        self.port = port
        self.client_name = client_name

        self.variables = IMU_VARIABLES + DVL_VARIABLES

        self.values = {}
        self.lock = threading.Lock()

        self.running = False
        self.sock = None
        self.thread = None

    def exchange(self, messages):

        self.sock.sendall(pack_packet(messages))

        return read_packet(self.sock)

    def store(self, messages):

        with self.lock:

            for message in messages:

                name = message["name"]

                if name == "NAV_Depth":
                    name = "NAV_DEPTH"

                if name not in self.variables:
                    continue

                try:
                    value = message_value(message)

                    if isinstance(value, (int, float)):
                        self.values[name] = float(value)

                except (TypeError, ValueError):
                    continue

    def registration_messages(self):

        return [
            pack_message(
                "R",
                name,
                source=self.client_name
            )
            for name in self.variables
        ]

    def start(self):

        self.sock = socket.create_connection(
            (self.host, self.port),
            timeout=10.0
        )

        self.sock.settimeout(5.0)

        self.sock.sendall(MOOS_PROTOCOL)

        welcome = self.exchange([
            pack_message(
                "i", "",
                text=self.client_name,
                data_type="S"
            )
        ])

        if welcome and welcome[0]["type"] == "K":
            raise RuntimeError(welcome[0]["text"])

        if not welcome or welcome[0]["type"] != "W":
            raise RuntimeError("Invalid MOOS handshake")

        self.store(
            self.exchange(self.registration_messages())
        )

        self.running = True

        self.thread = threading.Thread(
            target=self.collect,
            daemon=True
        )

        self.thread.start()

    def collect(self):

        try:

            while self.running:

                started = time.monotonic()

                messages = self.exchange([
                    pack_message(
                        ".",
                        "",
                        source=self.client_name
                    )
                ])

                self.store(messages)

                delay = 0.1 - (
                    time.monotonic() - started
                )

                if delay > 0:
                    time.sleep(delay)

        except Exception as exc:

            if self.running:
                print(
                    f"MOOS communication error: {exc}"
                )

            self.running = False

    def snapshot(self):

        with self.lock:
            return self.values.copy()

    def close(self):

        self.running = False

        if self.sock is not None:

            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

            self.sock.close()
            self.sock = None

        if (
            self.thread is not None
            and self.thread is not threading.current_thread()
        ):
            self.thread.join(timeout=2.0)

        self.thread = None


# ============================================================
# 3. ROS 2 节点
# ============================================================

class MoosSensorBridge(Node):

    def __init__(self):

        super().__init__("moos_sensor_bridge")

        # ---------------- ROS 参数 ----------------
        #host, port, client_name, publish_rate, imu_frame, dvl_frame
        self.declare_parameter(
            "host", "192.168.5.24"
        )

        self.declare_parameter(
            "port", 9000
        )

        self.declare_parameter(
            "client_name", "pMoosSensorBridge"
        )

        self.declare_parameter(
            "publish_rate", 10.0
        )

        self.declare_parameter(
            "imu_frame", "imu_link"
        )

        self.declare_parameter(
            "dvl_frame", "dvl_link"
        )

        host = self.get_parameter(
            "host"
        ).value

        port = self.get_parameter(
            "port"
        ).value

        client_name = self.get_parameter(
            "client_name"
        ).value

        rate = self.get_parameter(
            "publish_rate"
        ).value

        self.imu_frame = self.get_parameter(
            "imu_frame"
        ).value

        self.dvl_frame = self.get_parameter(
            "dvl_frame"
        ).value

        if rate <= 0:
            raise ValueError("publish_rate must be positive")

        # ---------------- ROS Publisher ----------------

        self.imu_pub = self.create_publisher(
            Imu,
            "/auv/imu",
            qos_profile_sensor_data
        )

        self.dvl_pub = self.create_publisher(
            TwistStamped,
            "/auv/dvl",
            qos_profile_sensor_data
        )

        # ---------------- MOOS Client ----------------

        self.moos = MoosSensorClient(
            host,
            port,
            client_name
        )

        self.moos.start()

        self.get_logger().info(
            f"Connected to MOOSDB: {host}:{port}"
        )

        # ---------------- ROS Timer ----------------

        self.timer = self.create_timer(
            1.0 / rate,
            self.publish_sensors
        )

    # ----------------------------------------------------
    # Euler -> Quaternion
    # ----------------------------------------------------

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):

        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)

        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)

        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )

    # ----------------------------------------------------
    # Publish IMU
    # ----------------------------------------------------

    def publish_imu(self, data, stamp):

        required = IMU_VARIABLES

        if not all(key in data for key in required):
            return

        msg = Imu()

        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame

        # Angular velocity: rad/s
        msg.angular_velocity.x = data["IMU_GX"]
        msg.angular_velocity.y = data["IMU_GY"]
        msg.angular_velocity.z = data["IMU_GZ"]

        # Linear acceleration: m/s^2
        msg.linear_acceleration.x = data["IMU_AX"]
        msg.linear_acceleration.y = data["IMU_AY"]
        msg.linear_acceleration.z = data["IMU_AZ"]

        # Euler angles: degrees -> radians
        roll = math.radians(data["IMU_ROLL"])
        pitch = math.radians(data["IMU_PITCH"])
        yaw = math.radians(data["IMU_HEADING"])

        q = self.euler_to_quaternion(
            roll, pitch, yaw
        )

        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        # Covariance unknown
        # Zero covariance means unknown covariance.
        self.imu_pub.publish(msg)

    # ----------------------------------------------------
    # Publish DVL
    # ----------------------------------------------------

    def publish_dvl(self, data, stamp):

        required = [
            "DVL_BI_tv",
            "DVL_BI_lv",
            "DVL_BI_nv",
        ]

        if not all(key in data for key in required):
            return

        msg = TwistStamped()

        msg.header.stamp = stamp
        msg.header.frame_id = self.dvl_frame

        # Linear velocity: m/s
        msg.twist.linear.x = data["DVL_BI_tv"]
        msg.twist.linear.y = data["DVL_BI_lv"]
        msg.twist.linear.z = data["DVL_BI_nv"]

        # DVL does not directly provide angular velocity
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.dvl_pub.publish(msg)

    # ----------------------------------------------------
    # Timer callback
    # ----------------------------------------------------

    def publish_sensors(self):

        data = self.moos.snapshot()

        stamp = self.get_clock().now().to_msg()

        self.publish_imu(data, stamp)
        self.publish_dvl(data, stamp)

    # ----------------------------------------------------
    # Shutdown
    # ----------------------------------------------------

    def destroy_node(self):

        self.get_logger().info(
            "Closing MOOS connection..."
        )

        self.moos.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = MoosSensorBridge()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(f"Node error: {exc}")

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
