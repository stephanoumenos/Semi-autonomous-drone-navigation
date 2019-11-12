#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from st5drone.msg import BehaviorStatus

# If you need to protect a variable by a mutex...
from multiprocessing import Lock

# Then, as usual
import numpy as np

# And for image processing
import cv2


class Behavior:

    def __init__(self, behavior_name) : # We are in thread #1 here.

        if behavior_name == "ping":
            raise ValueError("Ping is a reserved behavior name!")

        # let us declare attributes that can be retrieved easily in the different methods.
        self.name = behavior_name

        self.active = False         # this will be thread protected...
        self.active_mutex = Lock() # ... thanks to this mutex.

        self.behavior = rospy.Subscriber("/behavior", BehaviorStatus, self.behavior_callback, queue_size=1)
        self.publisher = rospy.Publisher("/behavior_status", BehaviorStatus, queue_size=1)

    # This allows for a safe reading of the data
    def get_status(self) :
        with self.active_mutex :
            copy = self.active
        return copy

    # This is in the main thread #1
    def loop(self):
        while not rospy.is_shutdown():
            # ratio_msg  = Float32()
            # ratio.data = self.get_data() # we get the data, being protected from thread #2 writings.
            # self.pub_ratio(ratio_msg)
            rospy.sleep(0.1) # we sleep for 100ms
    #print(self.name + " is active")
            
            
    def behavior_callback(self, msg):

        if msg.behavior_name == self.name:

            with self.active_mutex :
                previous_status = self.active
                self.active = msg.active
            
            if previous_status = False and msg.active == True:
                self.on_status_on()
            elif previous_status = True and msg.active == False:
                self.on_status_off()

        elif msg.behavior_name == "ping":
            message = BehaviorStatus()
            message.behavior_name = self.name
            message.active = self.active
            
            self.publisher.publish(message)
    
    def on_status_on(self):
        pass    
    def on_status_off(self):
        pass


if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('my_node')

    my_node = Behavior("HI")
    #my_node.loop()
    rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.

