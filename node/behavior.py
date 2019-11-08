#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from std_msgs.msg      import Float32, Empty
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import CompressedImage
from geometry_msgs.msg import Twist
from st5drone.msg import behavior

# If you need to protect a variable by a mutex...
from multiprocessing import Lock

# Then, as usual
import numpy as np

# And for image processing
import cv2


class Behavior :

    def __init__(self, behavior_name) : # We are in thread #1 here.

        # let us declare attributes that can be retrieved easily in the different methods.
        self.name = behavior_name
        
        self.data = None         # this will be thread protected...
        self.data_mutex = Lock() # ... thanks to this mutex.

        self.behavior = rospy.Subscriber("/behavior", behavior, self.behavior_callback, queue_size=1)

    # This allows for a safe reading of the data
    def get_data(self) :
        with self.data_mutex :
            copy = self.data
        return copy

    # This is in the main thread #1
    def loop(self):
        while not rospy.is_shutdown():
            
            # ratio_msg  = Float32()
            # ratio.data = self.get_data() # we get the data, being protected from thread #2 writings.
            # self.pub_ratio(ratio_msg)
            
            rospy.sleep(0.1) # we sleep for 100ms

    def behavior_callback(self, msg):
        print(msg.behavior_name)



if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('my_node')
    
    my_node = Behavior("HI")
    #my_node.loop()     
    rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.