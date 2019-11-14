#!/usr/bin/env python

import rospy

# import the message type you need (here are examples)
from std_msgs.msg import Float32, Empty, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# If you need to protect a variable by a mutex...
from multiprocessing import Lock


class SpeedController:
    def __init__(self):  # We are in thread #1 here.

        rospy.init_node('speed_controller')
        self.max_vertical_speed = rospy.get_param('~SpeedSettingsMaxVerticalSpeedCurrent', 1)
        self.max_rotation_speed = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 100)

        self.frequency = 10

        self.commanded_speed = Twist()
        self.normalized_speed = Twist()
        self.speed_mutex = Lock()

        self.odom = Odometry()
        self.odom_mutex = Lock()

        self.hovering = False

        self.sub_hovering = rospy.Subscriber('/hovering', Bool,
                                             self.update_hovering,
                                             queue_size=1)

        self.sub_linear_x = rospy.Subscriber("linear_x", Float32,
                                             self.update_linear_x,
                                             queue_size=10)
        self.sub_linear_y = rospy.Subscriber("linear_y", Float32,
                                             self.update_linear_y,
                                             queue_size=10)
        self.sub_linear_z = rospy.Subscriber("linear_z", Float32,
                                             self.update_linear_z,
                                             queue_size=10)
        self.sub_angular_z = rospy.Subscriber("angular_z", Float32,
                                              self.update_angular_z,
                                              queue_size=10)

        self.sub_odom = rospy.Subscriber("/bebop/odom", Odometry,
                                         self.update_odometry,
                                         queue_size=10)

        self.pub_speed = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        # Controller

        self.gains = {
            "linear_x": {'Tu' : 1.65, 'Ku' : 1.6, 'Kp': 0.0, 'Kd': 0.0, 'Ki': 0.0},
            "linear_y": {'Tu' : 1.6, 'Ku' : 1.6, 'Kp': 0.0, 'Kd': 0.0, 'Ki': 0.0}
        }
        self.no_overshoot()

        self.errors_last_update = 0

        self.errors = {
            "linear_x": {'error': 0.0, 'D_error': 0.0, 'I_error': 0.0, 'time': rospy.Time.now()},
            "linear_y": {'error': 0.0, 'D_error': 0.0, 'I_error': 0.0, 'time': rospy.Time.now()}
        }


    def some_overshoot(self):
        self.gains['linear_x']['Kp']=self.gains['linear_x']['Ku']/3
        self.gains['linear_x']['Kd']=self.gains['linear_x']['Ku']*self.gains['linear_x']['Tu']/9
        self.gains['linear_x']['Ki']=0.666*self.gains['linear_x']['Ku']/self.gains['linear_x']['Tu']

        self.gains['linear_y']['Kp']=self.gains['linear_y']['Ku']/3
        self.gains['linear_y']['Kd']=self.gains['linear_y']['Ku']*self.gains['linear_y']['Tu']/9
        self.gains['linear_y']['Ki']=0.666*self.gains['linear_y']['Ku']/self.gains['linear_y']['Tu']


    def pi_controller(self):
        self.gains['linear_x']['Kp']=0.45*self.gains['linear_x']['Ku']
        self.gains['linear_x']['Kd']=0
        self.gains['linear_x']['Ki']=0.54*self.gains['linear_x']['Ku']/self.gains['linear_x']['Tu']
        self.gains['linear_y']['Kp']=0.45*self.gains['linear_y']['Ku']
        self.gains['linear_y']['Kd']=0
        self.gains['linear_y']['Ki']=0.54*self.gains['linear_y']['Ku']/self.gains['linear_y']['Tu']


    def pid_classic(self):
        self.gains['linear_x']['Kp']=0.6*self.gains['linear_x']['Ku']
        self.gains['linear_x']['Kd']=3*self.gains['linear_x']['Ku']*self.gains['linear_x']['Tu']/40
        self.gains['linear_x']['Ki']=1.2*self.gains['linear_x']['Ku']/self.gains['linear_x']['Tu']

        self.gains['linear_y']['Kp']=0.6*self.gains['linear_y']['Ku']
        self.gains['linear_y']['Kd']=3*self.gains['linear_y']['Ku']*self.gains['linear_y']['Tu']/40
        self.gains['linear_y']['Ki']=1.2*self.gains['linear_y']['Ku']/self.gains['linear_y']['Tu']


    def no_overshoot(self):
        self.gains['linear_x']['Kp']=self.gains['linear_x']['Ku']/5
        self.gains['linear_x']['Kd']=self.gains['linear_x']['Ku']*self.gains['linear_x']['Tu']/15
        self.gains['linear_x']['Ki']=2/5*self.gains['linear_x']['Ku']/self.gains['linear_x']['Tu']

        self.gains['linear_y']['Kp']=self.gains['linear_y']['Ku']/5
        self.gains['linear_y']['Kd']=self.gains['linear_y']['Ku']*self.gains['linear_y']['Tu']/15
        self.gains['linear_y']['Ki']=2/5*self.gains['linear_y']['Ku']/self.gains['linear_y']['Tu']

    def update_hovering(self, msg):
        self.hovering = msg.data

    def publish_speed(self):
        self.pub_speed.publish(self.normalized_speed)

    def hover(self):
        self.pub_speed.publish(Twist())

    def update_linear_x(self, value):
        with self.speed_mutex:
            self.commanded_speed.linear.x = value.data

    def update_linear_y(self, value):
        with self.speed_mutex:
            self.commanded_speed.linear.y = value.data

    def update_linear_z(self, value):
        with self.speed_mutex:
            self.commanded_speed.linear.z = value.data

    def update_angular_z(self, value):
        with self.speed_mutex:
            self.commanded_speed.angular.z = value.data

    def loop(self):
        while not rospy.is_shutdown():
            if self.odom.header.stamp != self.errors_last_update:
                with self.odom_mutex:
                    self.update_errors()
                self.errors_last_update = self.errors['linear_x']['time']

            with self.speed_mutex:
                if self.hovering:
                    self.hover()
                else:
                    self.normalized_speed.linear.x = (self.gains['linear_x']['Kp']*self.errors['linear_x']['error'] + self.gains['linear_x']['Kd']*self.errors['linear_x']['D_error'] + self.gains['linear_x']['Ki']*self.errors['linear_x']['I_error'] ) / self.max_vertical_speed
                    self.normalized_speed.linear.y = (self.gains['linear_y']['Kp']*self.errors['linear_y']['error'] + self.gains['linear_y']['Kd']*self.errors['linear_y']['D_error'] + self.gains['linear_y']['Ki']*self.errors['linear_y']['I_error'] ) / self.max_vertical_speed
                    self.normalized_speed.linear.z = self.commanded_speed.linear.z / self.max_vertical_speed
                    self.normalized_speed.angular.z = self.commanded_speed.angular.z / self.max_rotation_speed
                    self.publish_speed()

            rospy.sleep(1 / self.frequency)  # we sleep for 100ms

    def update_odometry(self, data):
        with self.odom_mutex:
            self.odom = data

    def update_errors(self):
        last_errors = self.errors
        now = self.odom.header.stamp
        self.errors['linear_x']['error'] = self.commanded_speed.linear.x - self.odom.twist.twist.linear.x
        self.errors['linear_x']['D_error'] = (self.errors['linear_x']['error'] - last_errors['linear_x']['error']) \
                                             / (now - last_errors['linear_x']['time']).to_sec()
        self.errors['linear_x']['I_error'] = self.errors['linear_x']['I_error'] + \
                                             (self.errors['linear_x']['error'] + last_errors['linear_x']['error']) * \
                                             (now - last_errors['linear_x']['time']).to_sec() / 2
        self.errors['linear_x']['time'] = now

        self.errors['linear_y']['error'] = self.commanded_speed.linear.y - self.odom.twist.twist.linear.y
        self.errors['linear_y']['D_error'] = (self.errors['linear_y']['error'] - last_errors['linear_y']['error']) \
                                             / (now - last_errors['linear_y']['time']).to_sec()
        self.errors['linear_y']['I_error'] = self.errors['linear_y']['I_error'] + \
                                             (self.errors['linear_y']['error'] + last_errors['linear_y']['error']) * \
                                             (now - last_errors['linear_y']['time']).to_sec() / 2
        self.errors['linear_y']['time'] = now


if __name__ == '__main__':
    my_node = SpeedController()
    my_node.loop()
    # rospy.spin()  # useless... since loop already blocks. If you have
    # no idle job (i.e. loop) to do outside event
    # handling, rospy.spin() is mandatory in order to
    # prevent from immediate termination of your node.
