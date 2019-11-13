#!/usr/bin/env python2

import rospy
from behavior import Behavior

from std_msgs.msg import Float32


class Hover(Behavior):

    def __init__(self):
        super(Hover, self).__init__('Hover')
        self.pub_linear_x = rospy.Publisher('/linear_x', Float32, queue_size=50)
        self.pub_linear_y = rospy.Publisher('/linear_y', Float32, queue_size=50)
        self.pub_linear_z = rospy.Publisher('/linear_z', Float32, queue_size=50)
        self.pub_angular_z = rospy.Publisher('/angular_z', Float32, queue_size=50)

    def on_status_on(self):
        self.pub_linear_x.publish(0)
        self.pub_linear_y.publish(0)
        self.pub_linear_z.publish(0)
        self.pub_angular_z.publish(0)
        rospy.sleep(2)
        self.set_status(False)

    def on_status_off(self):
        pass


if __name__ == '__main__':
    node = Hover()
    rospy.spin()
