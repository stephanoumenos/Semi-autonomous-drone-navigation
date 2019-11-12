#!/usr/bin/env python

import rospy

from behavior import Behavior


if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('my_node')

    my_node = Behavior("b2")
    #my_node.loop()
    rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.
