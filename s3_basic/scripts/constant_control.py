#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# import the message type to ues
from std_msgs.msg import Int64, Bool, String

class ConstantPublisher(Node):
    def __init__(self) -> None:
        super().__init__("constant_publisher")
        self.msg_counter = 0
        self.cp_pub = self.create_publisher(String, "/constant_publisher", 10)
        self.cp_twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cp_timer = self.create_timer(0.2, self.cp_callback)

        self.cp_break_sub = self.create_subscription(Bool, "/kill", self.kill_callback_sub, 10)

    def cp_callback(self) -> None:
        msg = String()
        msg.data = f"sending constant control {self.msg_counter}"

        # Twist message
        msg_twist = Twist()
        msg_twist.linear.x = 2.5
        msg_twist.angular.z = 0.0

        # Publish
        self.cp_pub.publish(msg)
        self.cp_twist_pub.publish(msg_twist)

        # Increment
        self.msg_counter += 1

    def kill_callback_sub(self, msg: Bool) -> None:
        if msg.data:
            self.cp_timer.cancel()
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cp_twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher=ConstantPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

