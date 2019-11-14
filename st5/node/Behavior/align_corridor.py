#!/usr/bin/env python2

import rospy
from behavior import Behavior

from st5drone.msg import VanishPoint
from std_msgs.msg import Float32, Bool

from multiprocessing import Lock


class AlignCorridor(Behavior):

    def __init__(self):
        super(AlignCorridor, self).__init__('AlignCorridor')
        self.FOV = 90  # Careful experimentation (SICK)
        self.speed = 25
        self.last_vp = None
        self.vp_mutex = Lock()
        self.sub_vp = rospy.Subscriber("vanish_point", VanishPoint, self.on_vp_received, queue_size=1)
        self.pub_angular_z = rospy.Publisher('/angular_z', Float32, queue_size=50)
        self.pub_moving = rospy.Publisher('/moving', Bool, queue_size=0)

    def on_status_on(self):
        if self.last_vp is None:
            return

        self.publish_movement(True)
        with self.vp_mutex:
            distance_from_center = abs(self.last_vp.x - self.last_vp.res_x/2)
            rotation_angle = self.FOV * distance_from_center/self.last_vp.res_x
        maneuver_time = rotation_angle/self.speed
        self.pub_angular_z.publish(self.speed)
        rospy.sleep(maneuver_time)
        self.pub_angular_z.publish(0)
        self.set_status(False)

    def on_status_off(self):
        self.publish_movement(False)

    def publish_movement(self, moving):
        msg = Bool()
        msg.data = moving
        self.pub_moving.publish(msg)

    def on_vp_received(self, vp):
        with self.vp_mutex:
            self.last_vp = vp


if __name__ == '__main__':
    node = AlignCorridor()
    rospy.spin()
