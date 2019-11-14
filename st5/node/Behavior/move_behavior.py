#!/usr/bin/env python2

# Class that all movements are going to inherit from

import rospy
from behavior import Behavior

from std_msgs.msg import Float32, String, Bool


class Move(Behavior):

    def __init__(self, name, topic, rotational=False, backwards=False, duration=1):
        super(Move, self).__init__(name)
        max_vertical_speed = rospy.get_param('~SpeedSettingsMaxVerticalSpeedCurrent', 1)
        max_rotation_speed = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 100)
        self.TRANSLATIONAL_SPEED = 0.5 * max_vertical_speed
        self.ROTATIONAL_SPEED = 0.5 * max_rotation_speed
        self.MOVEMENT_DURATION = duration
        if rotational:
            self.movement_speed = self.ROTATIONAL_SPEED
        else:
            self.movement_speed = self.TRANSLATIONAL_SPEED
        if backwards:
            self.movement_speed *= -1
        self.pub = rospy.Publisher(topic, Float32, queue_size=0)
        # Publish when its moving so the controller stops
        self.pub_moving = rospy.Publisher('/moving', Bool, queue_size=0)
        self.pub_command = rospy.Publisher('/command', String, queue_size=0)

    def move(self):
        moving_msg = Bool()
        moving_msg.data = True
        self.pub_moving.publish(moving_msg)

        self.pub.publish(self.movement_speed)

    def stop(self):
        self.pub.publish(0)

        moving_msg = Bool()
        moving_msg.data = False
        self.pub_moving.publish(moving_msg)

        hover_msg = String()
        hover_msg.data = "Hover"
        self.pub_command.publish(hover_msg)

    def on_status_on(self):
        self.move()
        rospy.sleep(self.MOVEMENT_DURATION)
        self.set_status(False)

    def on_status_off(self):
        self.stop()

