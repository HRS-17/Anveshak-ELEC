#!/usr/bin/env python3

import rclpy as r
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ArmDriveNode(Node):

    def __init__(self):
        super().__init__("arm_drive")

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.joy_sub = self.create_subscription(Joy, "/joy_arm", self.joy_callback, self.qos)
        self.pub = self.create_publisher(Int32MultiArray, "/stm_write", self.qos)

        self.outbuff = [0] * 6


    def joy_callback(self, joy: Joy):
        outbuff = [0] * 6

        axes = [int(joy.axes[i] * 0xFF) for i in range(5)]
        buttons = [
            (joy.buttons[1] - joy.buttons[3]) * 255, 
            (joy.buttons[0] - joy.buttons[4]) * 255
        ]

        outbuff[0] = -axes[1]                  # Shoulder
        outbuff[1] = axes[0]                   # Base
        outbuff[2] = buttons[1] + buttons[0]   # buttons[0] is 2 buttons, say X and A. If both outbuff[2] and outbuff[5] are positive, roll (say) will happen clockwise; if they are -ve, it will happen anti-clockwise. If [2] is +ve and [5] is -ve, pitch will happen (say) up and vice versa
        outbuff[3] = axes[3]                   # Elbow 
        outbuff[4] = -axes[2]                  # Gripper
        outbuff[5] = buttons[1] - buttons[0]

        self.outbuff = outbuff
        self.get_logger().info(f"Outbuff: {self.outbuff}")

        self.send_msg(self.outbuff)


    def send_msg(self, buff):
        msg = Int32MultiArray()
        msg.data = buff[:]

       

        self.pub.publish(msg)
    

def main(args=None):
    r.init(args=args)
    node = ArmDriveNode()
    r.spin(node)
    node.destroy_node()
    r.shutdown()


if __name__ == "__main__":
    main()

        
