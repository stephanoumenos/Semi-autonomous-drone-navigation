#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    MoveLeft = Move('MoveLeft', '/linear_y', backwards=True)
    rospy.spin()
