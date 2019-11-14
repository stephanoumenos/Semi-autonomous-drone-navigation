#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    UTurn = Move('UTurn', '/angular_z', rotational=True, duration=2)
    rospy.spin()