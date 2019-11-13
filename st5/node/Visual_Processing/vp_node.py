#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

import vp_core as vp

class Test:
    
    def __init__(self):
        self.sub = rospy.Subscriber("image_in/compressed",  CompressedImage, self.on_image,  queue_size = 1, buff_size=2**22)
        self.pub = rospy.Publisher("image_out/compressed", CompressedImage, queue_size=1)
        
    def on_image(self, msg):
        compressed_in = np.fromstring(msg.data, np.uint8)
        frame         = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)

        # if time is required in your cumputation, consider the current time
        # as the one recorded in the message timestamps. This enables a coherent
        # time sequence when the images come from a ros bag, since ros bag execution
        # can be paused, run step by step, ...
        now = msg.header.stamp

        # Let us publish the gray image
        if self.pub.get_num_connections() > 0:
            #image = vp.io_test(frame, now)

            msg              = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format       = "jpeg"
            msg.data         = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            self.pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node('test')
    test = Test()
    rospy.spin()