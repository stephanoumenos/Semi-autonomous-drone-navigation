#!/usr/bin/env python2

import rospy
from behavior import Behavior

from std_msgs.msg import Empty


class Hover(Behavior):

    def __init__(self):
        super(Hover, self).__init__('Hover')
        self.pub_takeoff = rospy.Publisher('/autoflight/pause', Empty, queue_size=0)

    def on_status_on(self):
        self.pub_takeoff.publish(Empty())
        rospy.sleep(2)
        self.set_status(False)

    def on_status_off(self):
        pass


if __name__ == '__main__':
    node = Hover()
    rospy.spin()
