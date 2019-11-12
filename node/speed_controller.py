#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Twist

# If you need to protect a variable by a mutex...
from multiprocessing import Lock



class SpeedController:

    def __init__(self):  # We are in thread #1 here.

        rospy.init_node('speed_controller')
        self.max_vertical_speed = rospy.get_param('~SpeedSettingsMaxVerticalSpeedCurrent', 1)
        self.max_rotation_speed = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 100)

        self.frequency = 10

        self.commanded_speed = Twist()
        self.normalized_speed = Twist()
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

        self.pub_speed = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

    def publish_speed(self):
        self.pub_speed.publish(self.normalized_speed)

    def update_linear_x(self, value):
        with self.speed_mutex:
            self.commanded_speed.linear.x = value.data

    def update_linear_y(self, value):
        with self.speed_mutex:
            self.commanded_speed.linear.y = value.data

    def update_linear_z(self, value):
        with self.speed_mutex:
            self.commanded_speed.linear.z = value.data

    def update_angular_z(self, value):
        with self.speed_mutex:
            self.commanded_speed.angular.z = value.data

    def loop(self):
        while not rospy.is_shutdown():
            with self.speed_mutex:
                #self.normalized_speed.linear.x = self.commanded_speed.linear.x / self.max_vertical_speed
                #self.normalized_speed.linear.y = self.commanded_speed.linear.y / self.max_vertical_speed
                self.normalized_speed.linear.z = self.commanded_speed.linear.z / self.max_vertical_speed
                self.normalized_speed.angular.z = self.commanded_speed.angular.z / self.max_rotation_speed
                self.publish_speed()

            rospy.sleep(1/self.frequency)  # we sleep for 100ms


if __name__ == '__main__':
    my_node = SpeedController()
    my_node.loop()
    #rospy.spin()  # useless... since loop already blocks. If you have
    # no idle job (i.e. loop) to do outside event
    # handling, rospy.spin() is mandatory in order to
    # prevent from immediate termination of your node.
