#!/usr/bin/env python2

# Class that all movements are going to inherit from

import rospy
from behavior import Behavior

from std_msgs.msg import Float32


class Move(Behavior):

    def __init__(self, topic, translational_or_rotational):
        super(Move, self).__init__('Move')
        self.TRANSLATIONAL_SPEED = 0.5
        self.ROTATIONAL_SPEED = 50
        self.MOVEMENT_DURATION = 1
        if translational_or_rotational:
            self.movement_speed = self.TRANSLATIONAL_SPEED
        else:
            self.movement_speed = self.ROTATIONAL_SPEED
        self.pub = rospy.Publisher(topic, Float32, queue_size=0)

    def move(self):
        self.pub.publish(self.movement_speed)

    def stop(self):
        self.pub.publish(0)

    def on_status_on(self):
        self.move()
        rospy.sleep(self.MOVEMENT_DURATION)
        self.set_status(False)

    def on_status_off(self):
        self.stop()

