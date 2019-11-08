#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from std_msgs.msg      import Float32, Empty
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import CompressedImage
from geometry_msgs.msg import Twist

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

        # Let us define the publishers
        self.pub_image = rospy.Publisher("/image_out/compressed", CompressedImage, queue_size = 1)
        self.pub_ratio = rospy.Publisher("/ratio",                Float32,         queue_size = 1)

        # Let us define some subscribers. Each declaration created a thread.
        
        self.sub_image = rospy.Subscriber("/image_in/compressed",  CompressedImage,
                                          self.on_image,   # This is thread #2
                                          queue_size = 1, buff_size=2**22) # This buff_size tricks prevents from delay in image stream.
        self.sub_odom  = rospy.Subscriber("/odom", Odometry,
                                          self.on_odom,     # This is thread #3
                                          queue_size=1)

    # This called within thread #3 when an odometry message is recieved.
    def on_odom(self, msg):
        current_velocity = msg.twist.twist
        # ...

    # This called within thread #2 when an image message is recieved.
    def on_image(self, msg):
        # From ros message to cv image
        compressed_in = np.fromstring(msg.data, np.uint8)
        frame         = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)

        # Then we can play with opencv
        gray          = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        width         = gray.shape[1]
        height        = gray.shape[0]

        # Let us lock data in order to compute if from the frame
        with self.data_mutex :
            self.data = some_function(gray)

        # Let us publish the gray image
        if self.pub_image.get_num_connections() > 0:
            msg              = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format       = "jpeg"
            msg.data         = np.array(cv2.imencode('.jpg', gray)[1]).tostring()
            self.pub_img.publish(msg)

    # This allows for a safe reading of the data
    def get_data(self) :
        with self.data_mutex :
            copy = somehow_make_a_copy(self.data)
        return copy

    # This is in the main thread #1
    def loop(self):
        while not rospy.is_shutdown():
            
            ratio_msg  = Float32()
            ratio.data = self.get_data() # we get the data, being protected from thread #2 writings.
            self.pub_ratio(ratio_msg)
            
            rospy.sleep(0.1) # we sleep for 100ms



if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('my_node')

    my_node = MyNode()
    my_node.loop()     
    rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.