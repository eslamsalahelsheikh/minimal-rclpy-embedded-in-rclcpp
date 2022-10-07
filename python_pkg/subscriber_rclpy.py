#!/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class SpinThread(threading.Thread):
    def __init__(self, node):
        threading.Thread.__init__(self)
        self.node = node

    def run(self):
        rclpy.spin(self.node)

class MinimalSubscriber(Node):

    def __init__(self):
        rclpy.init(args=None)
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'Chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        SpinThread(self).start()
        print ("rclpy thread started")
        # rclpy.spin(self)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def __del__(self):
        self.destroy_node()
        rclpy.shutdown()
