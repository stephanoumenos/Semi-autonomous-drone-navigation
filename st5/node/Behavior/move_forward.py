#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    MoveForward = Move('MoveForward', '/linear_x')
    rospy.spin()
