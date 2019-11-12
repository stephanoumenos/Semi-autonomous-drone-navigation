#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math

# Buttons are things you can click on the joystick, while axes allow
# for gradual position.

BUTTON_A                  = 0 # A button.
BUTTON_B                  = 1 # B button.
BUTTON_X                  = 2 # X button.
BUTTON_Y                  = 3 # Y bttonu.
BUTTON_LEFT_FRONT         = 4 # The top left button on the front of the joystick.
BUTTON_RIGHT_FRONT        = 5 # The top right button on the front of the joystick.
BUTTON_BACK               = 6 # The 'back' button.
BUTTON_SELECT             = 7 # The 'select' button.
BUTTON_LOGITECH           = 8 # The 'logitech' button.
BUTTON_CLICK_LEFT_PAD     = 9 # Left pad can be clicked when you press it from above.
BUTTON_CLICK_RIGHT_PAD    = 10 # Right pad can be clicked when you press it from above.

AXIS_LEFT_PAD_HORIZONTAL  = 0 # The horizontal position of the left pad.
AXIS_LEFT_PAD_VERTICAL    = 1 # The vertical position of the left pad.
AXIS_LEFT_FRONT           = 2 # The top right axis-button on the front of the joystick.
AXIS_RIGHT_PAD_HORIZONTAL = 3 # The horizontal position of the right pad.
AXIS_RIGHT_PAD_VERTICAL   = 4 # The vertical position of the right pad.
AXIS_RIGHT_FRONT          = 5 # The top right axis-button on the front of the joystick.
AXIS_CROSS_HORIZONTAL     = 6 # The horizontal position of the left cross.
AXIS_CROSS_VERTICAL       = 7 # The horizontal position of the right cross.


class JoyTeleop:

    def __init__(self):
        self.sub_joy = rospy.Subscriber("joy", Joy, self.on_joy, queue_size=1)
        self.cmd_pub = rospy.Publisher("command", String, queue_size=1)
        self.axis_tolerance = 0.75
        self.flying = False

    def send_command(self, cmd):
        cmd_msg = String()
        cmd_msg.data = cmd
        self.cmd_pub.publish(cmd_msg)

    # Tells wether an axis is significantly on one of its extremum value
    def axis_activated(self, msg, axis_id, direction):
        return direction * msg.axes[axis_id] > self.axis_tolerance

    # Tells wether an axis is at rest.
    def axis_released(self, msg, axis_id) :
        math.fabs(msg.axes[axis_id]) < 1.0 - self.axis_tolerance

    # Tells wether an axis which as a perpendicular axis associated
    # with it is significantly on one of its extremum value.
    def axis_activated_orthogonal(self, msg, axis_id, orthogonal_axis_id, direction):
        return self.axis_activated(msg, axis_id, direction) and self.axis_released(msg, orthogonal_axis_id);

    def axis_high(self, msg, axis_id) :
        return self.axis_activated(msg, axis_id, +1)

    def axis_low(self, msg, axis_id) :
        return self.axis_activated(msg, axis_id, -1)

    def axis_high_orthogonal(self, msg, axis_id, orthogonal_axis_id) :
        return self.axis_activated_orthogonal(msg, axis_id, orthogonal_axis_id, +1)

    def axis_low_orthogonal(self, msg, axis_id, orthogonal_axis_id) :
        return self.axis_activated_orthogonal(msg, axis_id, orthogonal_axis_id, -1)

    def clicked(self, msg, button_id) :
        return msg.buttons[button_id]

    def on_joy(self, msg):

        # Let us implement a deadman button security. The deadman
        # button is the bottom right button at the front of the
        # joystick. Keeping it pressed makes the drone keep on flying.

        if self.axis_low(msg, AXIS_RIGHT_FRONT) :
            if not self.flying:
                self.send_command("TakeOff") # Taking off and then hover.
                self.flying = True
            else:
                self.process_command(msg)
        elif self.flying :
            # Otherwise we land
            self.send_command("Land")
            self.flying = False

    def process_command(self, msg) :

        # Use one of the following method to detect the user actions and send a command accordingly.
        # - self.axis_low(msg, AXIS_*)
        # - self.axis_high(msg, AXIS_*)
        # - self.axis_low_orthogonal(msg, AXIS_*, AXIS_*)
        # - self.axis_high_orthogonal(msg, AXIS_*, AXIS_*)
        # - self.clicked(msg, BUTTON_*)

        if self.clicked(msg, BUTTON_B) :
            self.send_command("Hover")

        # add yours hereafter...

if __name__ == '__main__':
    rospy.init_node('joy_teleop')

    joy_teleop = JoyTeleop()

    rospy.spin()
