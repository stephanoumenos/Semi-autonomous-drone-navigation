#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32


# Expected input sizes
AXES_SIZE = 8
BUTTONS_SIZE = 11


class JoyMap():

    def __init__(self):
        #self.SCALE_LINEAR = rospy.get_param('~scale_linear', 0.26)
        #self.SCALE_ANGULAR = rospy.get_param('~scale_angular', 1.82)
        self.BRAKE_BUTTON = rospy.get_param('~brake_button', 0)
        #self.AXIS_LINEAR = rospy.get_param('~axis_linear', 1)
        #self.AXIS_ANGULAR = rospy.get_param('~axis_angular', 1)
        self.max_vertical_speed = rospy.get_param('~SpeedSettingsMaxVerticalSpeedCurrent', 0)
        self.max_rotation_speed = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 0)
        # If RT or LT is pressed more than 50%, it means it is pressed
        self.PRESSED_THRESHOLD = rospy.get_param('~pressed_threshold', 0.9)
        rospy.init_node('JoyMap', anonymous=True)

        # Publish the drone movements
        #self.pub_linear_x = rospy.Publisher('/linear_x', Float32, queue_size=50)
        #self.pub_linear_y = rospy.Publisher('/linear_y', Float32, queue_size=50)
        self.pub_linear_z = rospy.Publisher('/linear_z', Float32, queue_size=50)
        self.pub_angular_z = rospy.Publisher('/angular_z', Float32, queue_size=50)
        #self.pub_mov = rospy.Publisher('/target_vel', Twist, queue_size=50)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=0)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=0)

        self.sub = rospy.Subscriber('/joy', Joy, callback=self.publishMappedVelocities)

        # Buttons

        # Emergency
        self.A = None

        self.RT = None
        self.left_stick_horizontal = None
        self.left_stick_vertical = None
        self.right_stick_horizontal = None
        self.right_stick_vertical = None

        # Used to control landing / take off
        self.last_command = None

    def axisToPercentage(self, axis):
        # RT and LT start in 1 and go to -1 when fully pressed
        # axis [1, -1] -> [0, 1]
        return (-axis+1)/2

    def invertStick(self, stick):
        # sticks go from 1 (left) to -1 (right)
        # Stick [1, -1] -> [-1, 1]
        return -stick


    def mapper(self, axesArray, buttonArray):

        # Landing / take-off
        self.RT = self.axisToPercentage(axesArray[5])

        # A to break
        self.A = buttonArray[self.BRAKE_BUTTON]

        # Left Stick
        self.left_stick_horizontal = axesArray[0]
        self.left_stick_vertical = axesArray[1]
        self.right_stick_horizontal = self.invertStick(axesArray[3])
        self.right_stick_vertical = axesArray[4]

    def take_off(self):
        print("TAKE OFF")
        self.pub_takeoff.publish(Empty())

    def land(self):
        print("LAND")
        self.pub_land.publish(Empty())

    def publishMappedVelocities(self, data):
        """
        Used as a callback to publish the mapped velocities
        to the /cmd_vel_mux/input/teleop topic
        """
        if len(data.axes) < AXES_SIZE or len(data.buttons) < BUTTONS_SIZE:
            print '[JOY_MAP] Unexpected input format. Expected %s axes_size got %s' \
                   'and expected %s buttons_size got %s' % (AXES_SIZE, len(data.axes), BUTTONS_SIZE, len(data.buttons))
            return
        self.mapper(data.axes, data.buttons)

        # Land / Take-Off
        assert self.RT is not None
        assert self.A is not None

        if self.A == 1:
            self.land()
            return

        if self.RT >= self.PRESSED_THRESHOLD:
            if self.last_command != 'take_off':
                self.take_off()
                self.last_command = 'take_off'
                return
        else:
            if self.last_command != 'land':
                self.land()
                self.last_command = 'land'
                return

        assert (self.left_stick_horizontal is not None and
                self.left_stick_vertical is not None and
                self.right_stick_horizontal is not None and
                self.right_stick_vertical is not None)

        # Translate forward / back
        #vel_msg.linear.x = self.left_stick_vertical
        # Translate left / right
        #vel_msg.linear.y = self.left_stick_horizontal
        # Ascend / descend
        self.pub_linear_z.publish(self.right_stick_vertical * self.max_vertical_speed)
        #vel_msg.linear.z = self.right_stick_vertical
        # Rotate
        self.pub_angular_z.publish(self.right_stick_horizontal * self.max_rotation_speed)
        #vel_msg.angular.z = self.right_stick_horizontal
        #print(vel_msg)
        #self.pub_mov.publish(vel_msg)


    def Spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = JoyMap()
    node.Spin()
