#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

# If you need to protect a variable by a mutex...
from multiprocessing import Lock

# Then, as usual
import numpy as np

# And for image processing
import cv2


class SpeedController:

    def __init__(self):  # We are in thread #1 here.

        self.frequency = 10

        self.speed = Twist()
        self.speed_mutex = Lock()  # ... thanks to this mutex.

        self.sub_linear_x = rospy.Subscriber("linear_x", Float32,
                                             self.update_linear_x,
                                             queue_size=10)
        self.sub_linear_y = rospy.Subscriber("linear_y", Float32,
                                             self.update_linear_y,
                                             queue_size=10)
        self.sub_linear_z = rospy.Subscriber("linear_z", Float32,
                                             self.update_linear_z,
                                             queue_size=10)
        self.sub_angular_z = rospy.Subscriber("angular_z", Float32,
                                              self.update_angular_z,
                                              queue_size=10)

        self.pub_speed = rospy.Publisher('/target_vel', Twist, queue_size=10)

    def publish_speed(self):
        self.pub_speed.publish(self.speed)

    def update_linear_x(self, value):
        with self.data_mutex:
            self.speed.linear.x = value.data
        self.publish_speed()

    def update_linear_y(self, value):
        with self.data_mutex:
            self.speed.linear.y = value.data
        self.publish_speed()

    def update_linear_z(self, value):
        with self.data_mutex:
            self.speed.linear.z = value.data
        self.publish_speed()

    def update_angular_z(self, value):
        with self.data_mutex:
            self.speed.angular.z = value.data
        self.publish_speed()

    def loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(1/self.frequency)  # we sleep for 100ms


if __name__ == '__main__':
    rospy.init_node('speed_controller')
    my_node = SpeedController()
    rospy.spin()  # useless... since loop already blocks. If you have
    # no idle job (i.e. loop) to do outside event
    # handling, rospy.spin() is mandatory in order to
    # prevent from immediate termination of your node.
