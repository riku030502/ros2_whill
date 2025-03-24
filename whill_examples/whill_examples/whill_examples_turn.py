# Copyright (c) 2024 WHILL, Inc.
# Released under the MIT license
# https://opensource.org/licenses/mit-license.php

import sys
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from whill_msgs.srv import SetPower

class WhillClient(Node):

    def __init__(self):
        super().__init__('turn_demo_client')
        self.cli = self.create_client(SetPower, '/whill/set_power_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetPower.Request()

        self.joy_pub = self.create_publisher(Joy, '/whill/controller/joy', 10)

    def send_power_on(self):
        self.req.p0 = 1
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_power_off(self):
        self.req.p0 = 0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_joystick(self, joy_x, joy_y):
        msg = Joy(axes=[joy_x, joy_y])
        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    time.sleep(1)
    client = WhillClient()

    # power on
    response = client.send_power_on()
    if response.result != 1:
        client.get_logger().error('send_power_on() error.')
        return
    time.sleep(2)

    # turn left
    for i in range(50):
        client.send_joystick(0.2, 0.0)
        time.sleep(0.09)
    time.sleep(1)

    # turn right
    for i in range(50):
        client.send_joystick(-0.2, 0.0)
        time.sleep(0.09)
    time.sleep(1)

    # power off
    response = client.send_power_off()
    if response.result != 1:
        client.get_logger().error('send_power_on() error.')
        return

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()