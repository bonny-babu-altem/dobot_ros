import json
import threading
import queue
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cr5_message_type.msg import DigitalInOut, JointPosition, JointSpeed, TcpPosition
from cr5_robot_status_publisher.tcp_client import TCPMonitorClient

rclpy.init()

robot_status = Node(node_name="robot_status_publisher")
digital_input_publisher = robot_status.create_publisher(
    DigitalInOut, "/digital_input", 10)
digital_output_publisher = robot_status.create_publisher(
    DigitalInOut, "/digital_output", 10)
joint_position_publisher = robot_status.create_publisher(
    JointPosition, "/joint_position", 10)
joint_speed_publisher = robot_status.create_publisher(
    JointSpeed, "/joint_speed", 10)
tcp_position_publisher = robot_status.create_publisher(
    TcpPosition, "/tcp_position", 10)


tcp_client = TCPMonitorClient(ip="192.168.0.116")
tcp_client.connect()


def publish_status(data) -> None:

    # Publish DI
    di_msg = DigitalInOut()
    for i, val in enumerate(data["digital_inputs"], start=1):
        setattr(di_msg, f"digital_{i}", val)
    digital_input_publisher.publish(di_msg)

    # Publish DO
    do_msg = DigitalInOut()
    for i, val in enumerate(data["digital_outputs"], start=1):
        setattr(do_msg, f"digital_{i}", val)
    digital_output_publisher.publish(do_msg)

    # Publish joint positions
    jp_msg = JointPosition()
    for i, val in enumerate(data["joints"]["position"], start=1):
        setattr(jp_msg, f"joint_{i}", val)
    joint_position_publisher.publish(jp_msg)

    # Publish joint speeds
    js_msg = JointSpeed()
    for i, val in enumerate(data["joints"]["speed"], start=1):
        setattr(js_msg, f"joint_{i}", val)
    joint_speed_publisher.publish(js_msg)

    # Publish TCP position
    tcp_fields = ["x_cord", "y_cord", "z_cord",
                  "rx_cord", "ry_cord", "rz_cord"]
    tcp_msg = TcpPosition()
    for field, val in zip(tcp_fields, data["tcp"]):
        setattr(tcp_msg, field, val)
    tcp_position_publisher.publish(tcp_msg)


def publish_data():
    while True:
        packet = tcp_client.read_packet()
        data = tcp_client.parse_packet(packet)
        publish_status(data)


def main(args=None):
    thread = threading.Thread(target=publish_data, daemon=True)
    try:
        thread.start()
        rclpy.spin(robot_status)
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        thread.join()
        tcp_client.disconnect()
        rclpy.shutdown()
        robot_status.destroy_node()
