#!/usr/bin/env python2
from move_behavior import Move

import rospy

if __name__ == '__main__':
    MoveUp = Move('MoveUp', '/linear_z', 'translational')
    rospy.spin()
