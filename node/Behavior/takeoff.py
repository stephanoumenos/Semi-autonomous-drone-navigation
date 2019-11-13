#!/usr/bin/env python2

import rospy
from behavior import Behavior

from std_msgs.msg import Empty


class TakeOff(Behavior):

    def __init__(self):
        super(TakeOff, self).__init__('TakeOff')
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=0)

    def on_status_on(self):
        self.pub_takeoff.publish(Empty())
        rospy.sleep(2)
        self.set_status(False)

    def on_status_off(self):
        pass


if __name__ == '__main__':
    node = TakeOff()
    rospy.spin()

