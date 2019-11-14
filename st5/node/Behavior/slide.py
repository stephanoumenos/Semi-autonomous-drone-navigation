#!/usr/bin/env python2

import rospy
from behavior import Behavior

from multiprocessing import Lock

from std_msgs.msg import String
from nav_msgs.msg import Odometry

import numpy as np


class Slide(Behavior):

    def __init__(self):
        super(Slide, self).__init__('Slide')
        self.pub_command = rospy.Publisher('/command', String, queue_size=50)
        self.odom = Odometry()
        self.odom_mutex = Lock()
        self.initial_orientation = None
        self.initial_position = None
        self.axis = None
        self.sub_odom = rospy.Subscriber("/bebop/odom", Odometry,
                                         self.update_odometry,
                                         queue_size=10)

    def send_command(self, command):
        command_msg = String()
        command_msg.data = command
        self.pub_command.publish(command_msg)

    def update_odometry(self, data):
        with self.odom_mutex:
            self.odom = data

    def store_forward_axis(self):
        self.initial_position = self.odom.pose.pose.position
        self.initial_orientation = self.odom.pose.pose.orientation
        # This is the vector pointing forward the robot
        self.axis = np.array([self.initial_orientation.w ** 2 + self.initial_orientation.x ** 2
                              - self.initial_orientation.y ** 2 - self.initial_orientation.z ** 2,
                              2 * self.initial_orientation.w * self.initial_orientation.z +
                              2 * self.initial_orientation.x * self.initial_orientation.y])

    def on_status_on(self):
        self.store_forward_axis()

    def calculate_normal(self):
        od = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])
        op_zero = np.array([self.initial_position.x, self.initial_position.y])
        normal = (np.array([[1,0],[0,1]])-np.array([[self.axis[1]**2,self.axis[1]*self.axis[2]],[self.axis[1]*self.axis[2],self.axis[2]**2]]))*(od - op_zero)

    def on_status_off(self):
        pass


if __name__ == '__main__':
    node = Slide()
    rospy.spin()
