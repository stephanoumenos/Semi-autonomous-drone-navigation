#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    RotateRight = Move('RotateRight', '/angular_z', rotational=True, backwards=True)
    rospy.spin()
