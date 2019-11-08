#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


# Expected input sizes
AXES_SIZE = 8
BUTTONS_SIZE = 15


class JoyMap():

    def __init__(self):
        self.SCALE_LINEAR = rospy.get_param('~scale_linear', 0.26)
        self.SCALE_ANGULAR = rospy.get_param('~scale_angular', 1.82)
        self.BRAKE_BUTTON = rospy.get_param('~brake_button', 0)
        self.AXIS_LINEAR = rospy.get_param('~axis_linear', 1)
        self.AXIS_ANGULAR = rospy.get_param('~axis_angular', 1)
        rospy.init_node('JoyMap', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=50)
        self.sub = rospy.Subscriber('/joy', Joy, callback=self.publishMappedVelocities)


    def transformAxisToVelocity(self, axis):
        # RT and LT start in 1 and go to -1 when fully pressed
        # RT  [1, -1] -> [0, SCALE_LINEAR]
        return self.SCALE_LINEAR/2 * (-axis) / self.AXIS_LINEAR


    def transformStickToRotation(self, stick):
        # Horizontally, sticks go from 1 (left) to -1 (right)
        # Stick [1, -1] -> [-SCALE_ANGULAR, SCALE_ANGULAR]
        return -stick * self.SCALE_ANGULAR / self.AXIS_ANGULAR


    def mapper(self, axesArray, buttonArray):
        # RT to accelerate
        RT = axesArray[5]

        # LT to reversegg
        LT = axesArray[2]

        # A to break
        A = buttonArray[self.BRAKE_BUTTON]

        # Left Stick to rotate
        LR_stick = axesArray[0]
        #UD_stick = axesArray[1]

        finalAxisInput = RT - LT

        if A == 1:
            linear_velocity = 0
            angular_velocity = 0
        else:
            linear_velocity = self.transformAxisToVelocity(finalAxisInput)
            angular_velocity = self.transformStickToRotation(LR_stick)

        return (linear_velocity, angular_velocity)


    def publishMappedVelocities2(self, data):
        """
        Used as a callback to publish the mapped velocities
        to the /cmd_vel_mux/input/teleop topic
        """
        if len(data.axes) < AXES_SIZE or len(data.buttons) < BUTTONS_SIZE:
            print '[JOY_MAP] Unexpected input format. Expected %s axes_size got %s' \
                   'and expected %s buttons_size got %s' % (AXES_SIZE, len(data.axes), BUTTONS_SIZE, len(data.buttons))
            return
        vel_msg = Twist()
        linear_velocity, angular_velocity = self.mapper(data.axes, data.buttons)
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.pub.publish(vel_msg)


    def Spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = JoyMap()
    node.Spin()
