#!/usr/bin/env python2

import rospy
from behavior import Behavior

from st5drone.msg import VanishPoint
from std_msgs.msg import Float32, Bool

from multiprocessing import Lock


class CenterCorridor(Behavior):

    def __init__(self):
        super(CenterCorridor, self).__init__('CenterCorridor')
        self.maneuver_time = 0.5
        self.speed = 0.08  # 8 cm
        self.last_vp = None
        self.vp_mutex = Lock()
        self.sub_vp = rospy.Subscriber("vanish_point", VanishPoint, self.on_vp_received, queue_size=0)
        self.pub_linear_y = rospy.Publisher('/linear_y', Float32, queue_size=50)
        self.pub_moving = rospy.Publisher('/moving', Bool, queue_size=0)

    def on_status_on(self):
        if self.last_vp is None:
            return

        with self.vp_mutex:
            left_angle = self.last_vp.random_angle_left
            right_angle = self.last_vp.random_angle_right

        if left_angle == right_angle:
            pass
        elif left_angle > right_angle:  # Drone is closer to the left wall
            self.move_a_bit('right')
            self.publish_movement(True)
        else:
            self.move_a_bit('left')
            self.publish_movement(True)

        rospy.sleep(self.maneuver_time)
        self.set_status(False)

    def publish_movement(self, moving):
        msg = Bool()
        msg.data = moving
        self.pub_moving.publish(msg)

    def on_status_off(self):
        self.pub_linear_y.publish(0)
        self.publish_movement(False)

    def on_vp_received(self, vp):
        with self.vp_mutex:
            self.last_vp = vp

    def move_a_bit(self, direction):
        self.pub_linear_y.publish(self.speed if direction == 'right' else -self.speed)


if __name__ == '__main__':
    node = CenterCorridor()
    rospy.spin()