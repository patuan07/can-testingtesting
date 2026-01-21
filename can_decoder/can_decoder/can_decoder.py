import socket
import struct

import rclpy
from rclpy.node import Node

from can_receiver.msg import ImuPressure


CAN_IFACE = "can0"

CAN_ID_ROLL     = 0x300
CAN_ID_PITCH    = 0x301
CAN_ID_YAW      = 0x302
CAN_ID_PRESSURE = 0x303
CAN_ID_DEPTH    = 0x304

SCALE_RPY = 10000.0
SCALE_PD  = 100.0


def u16_le(b0, b1):
    return b0 | (b1 << 8)

def i16_le(b0, b1):
    v = u16_le(b0, b1)
    return v - 0x10000 if v & 0x8000 else v

def i32_le(b0, b1, b2, b3):
    v = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    return v - 0x100000000 if v & 0x80000000 else v


class CanPublisher(Node):

    def __init__(self):
        super().__init__("can_publisher")

        self.publisher = self.create_publisher(
            ImuPressure,
            "can/imu_pressure",
            10
        )

        self.sock = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
        )
        self.sock.bind((CAN_IFACE,))

        self.latest = ImuPressure()

        # non-blocking read loop at ~1 kHz
        self.timer = self.create_timer(0.001, self.read_can)

        self.get_logger().info("CAN publisher started")

    def read_can(self):
        try:
            frame = self.sock.recv(16)
        except BlockingIOError:
            return

        can_id, dlc, data = struct.unpack("=IB3x8s", frame)
        can_id &= 0x1FFFFFFF
        data = data[:dlc]

        if dlc != 8:
            return

        b = list(data)

        if can_id == CAN_ID_ROLL:
            self.latest.roll = i32_le(*b[:4]) / SCALE_RPY
            self.latest.ctr_roll = u16_le(b[4], b[5])

        elif can_id == CAN_ID_PITCH:
            self.latest.pitch = i32_le(*b[:4]) / SCALE_RPY
            self.latest.ctr_pitch = u16_le(b[4], b[5])

        elif can_id == CAN_ID_YAW:
            self.latest.yaw = i32_le(*b[:4]) / SCALE_RPY
            self.latest.ctr_yaw = u16_le(b[4], b[5])

        elif can_id == CAN_ID_PRESSURE:
            self.latest.pressure = u16_le(b[0], b[1]) / SCALE_PD
            self.latest.ctr_pressure = u16_le(b[2], b[3])

        elif can_id == CAN_ID_DEPTH:
            self.latest.depth = i16_le(b[0], b[1]) / SCALE_PD
            self.latest.ctr_depth = u16_le(b[2], b[3])

        else:
            return

        self.publisher.publish(self.latest)


def main():
    rclpy.init()
    node = CanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
