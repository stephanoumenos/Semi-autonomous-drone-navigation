#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    MoveDown = Move('MoveDown', '/linear_z', 'translational', backwards=True)
    rospy.spin()
