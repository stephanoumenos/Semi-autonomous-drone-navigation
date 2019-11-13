#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    MoveBackwards = Move('MoveBackwards', '/linear_x', 'translational', backwards=True)
    rospy.spin()
