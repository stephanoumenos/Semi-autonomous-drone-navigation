#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from st5drone.msg import BehaviorStatus

class Listener:

    def __init__(self, behavior_name) : # We are in thread #1 here.
        self.publisher = rospy.Subscriber("/behavior", BehaviorStatus, self.behavior_callback, queue_size=1)

    # This allows for a safe reading of the data
    def behavior_callback(self, msg):
        if msg.behavior_name != "ping":
            print(msg)
    
if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('listener')

    listener = Listener("HI")
    #my_node.loop()
    rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.

