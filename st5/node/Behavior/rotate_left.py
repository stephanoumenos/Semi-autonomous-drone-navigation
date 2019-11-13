#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    RotateLeft = Move('RotateLeft', '/angular_z', rotational=True)
    rospy.spin()
