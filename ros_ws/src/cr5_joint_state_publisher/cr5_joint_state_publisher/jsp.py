import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from cr5_message_type.msg import JointPosition


class JointPositionToJSP(Node):
    def __init__(self):
        super().__init__('joint_position_to_jsp')

        # Publisher for standard JointState
        self.jsp_pub = self.create_publisher(
            JointState, '/joint_states', QoSProfile(depth=10))

        # Subscriber for your custom joint position topic
        self.subscription = self.create_subscription(
            JointPosition,
            '/joint_position',
            self.joint_position_callback,
            QoSProfile(depth=10)
        )

        # Define joint names in order
        self.joint_names = [f'joint_{i+1}' for i in range(6)]

    def joint_position_callback(self, msg: JointPosition):
        jsp_msg = JointState()
        jsp_msg.header.stamp = self.get_clock().now().to_msg()
        jsp_msg.name = self.joint_names
        jsp_msg.position = [
            msg.joint_1,
            msg.joint_2,
            msg.joint_3,
            msg.joint_4,
            msg.joint_5,
            msg.joint_6,
        ]
        # You can leave velocity and effort empty or set later if needed
        jsp_msg.velocity = []
        jsp_msg.effort = []

        self.jsp_pub.publish(jsp_msg)
        self.get_logger().debug(f'Published JointState: {jsp_msg.position}')


def main(args=None):
    rclpy.init(args=args)
    node = JointPositionToJSP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
