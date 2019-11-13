#!/usr/bin/env python2

# Class that all movements are going to inherit from

import rospy
from behavior import Behavior

from std_msgs.msg import Float32, String


class Move(Behavior):

    def __init__(self, name, topic, rotational=False, backwards=False):
        super(Move, self).__init__(name)
        self.TRANSLATIONAL_SPEED = 0.5
        self.ROTATIONAL_SPEED = 50
        self.MOVEMENT_DURATION = 1
        if rotational:
            self.movement_speed = self.ROTATIONAL_SPEED
        else:
            self.movement_speed = self.TRANSLATIONAL_SPEED
        if backwards:
            self.movement_speed *= -1
        self.pub = rospy.Publisher(topic, Float32, queue_size=0)
        self.pub_command = rospy.Publisher('/command', String, queue_size=0)

    def move(self):
        self.pub.publish(self.movement_speed)

    def stop(self):
        self.pub.publish(0)
        hover_msg = String()
        hover_msg.data = "hover"
        self.pub_hover.publish(hover_msg)

    def on_status_on(self):
        self.move()
        rospy.sleep(self.MOVEMENT_DURATION)
        self.set_status(False)

    def on_status_off(self):
        self.stop()

