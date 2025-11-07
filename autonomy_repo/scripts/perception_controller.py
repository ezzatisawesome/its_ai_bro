#!/usr/bin/env python3
import rclpy
import time
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl

class PerceptionController(BaseController):
    def __init__(self, node_name: str = "perception_controller"):
        super().__init__(node_name)
        self.declare_parameter("active", True)
        self.start = None
    
    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
    
    def compute_control(self) -> TurtleBotControl:
        command = TurtleBotControl()
        if self.active:
            command.v = 0.0
            command.omega = 0.5
            return command
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if not self.start:
            self.start = current_time
        
        elapsed_time = current_time - self.start
        
        if elapsed_time >= 5.0:
            self.set_parameters([rclpy.Parameter("active", value=True)])
            self.start = None
            command.v = 0.0
            command.omega = 0.5
            return command
            
        command.v = 0.0
        command.omega = 0.0
        return command    

if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()