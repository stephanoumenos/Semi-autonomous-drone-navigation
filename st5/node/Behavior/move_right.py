#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    MoveRight = Move('MoveRight', '/linear_y')
    rospy.spin()
