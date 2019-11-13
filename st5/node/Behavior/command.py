#!/usr/bin/env python

import rospy
from st5drone.msg import BehaviorStatus
from std_msgs.msg import String
from event_queue import EventQueue

from multiprocessing import Lock

import sched, time

class Command:

    def __init__(self):
        self.behaviors = ['Hover', 'MoveLeft', 'MoveForward', 'MoveBackwards', 'MoveUp', 'MoveDown', 'MoveRight',
                          'RotateLeft', 'RotateRight', 'TakeOff', 'Land']
        self.commands = {
            'Stop': [(0, 'Hover')],
            'Dance': [(0, 'MoveLeft'), (0, 'MoveUp'), (2.5, 'MoveRight'), (3.2, 'Hover')],
            'TakeOff': [(0, 'TakeOff'), (1, 'Hover')],
            'Land': [(0, 'Land')],
            'Hover': [(0, 'Hover')],
            'MoveForward': [(0, 'MoveForward')],
            'MoveBackwards': [(0, 'MoveBackwards')]
            'MoveLeft': [(0, 'MoveLeft')],
            'MoveRight': [(0, 'MoveRight')],
            'MoveDown': [(0, 'MoveDown')],
            'MoveUp': [(0, 'MoveUp')],
            'RotateLeft': [(0, 'RotateLeft')],
            'RotateRight': [(0, 'RotateRight')]
        }

        self.command = rospy.Subscriber("/command", String, self.command_callback, queue_size=1)
        self.behavior = rospy.Publisher("/behavior", BehaviorStatus, queue_size=1)

        self.queue_lock = Lock()

        self.event_queue = EventQueue()

    def command_callback(self, msg):
        print("Entered callback")
        for behavior in self.behaviors:
            message = BehaviorStatus()
            message.behavior_name = behavior
            message.active = False
            # print(message)
            self.behavior.publish(message)

        with self.queue_lock:
            self.event_queue.setEvents(self.commands[msg.data])

    def contact_behavior(self, behavior_name):
    
        message = BehaviorStatus()
        message.behavior_name = behavior_name
        message.active = True

        self.behavior.publish(message)

    def loop(self):
        while not rospy.is_shutdown():
            with self.queue_lock:
                validEvents = self.event_queue.returnValidEvents()

            for behavior in validEvents:
                self.contact_behavior(behavior[1])

            rospy.sleep(0.1)


if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('Command')

    my_node = Command()

    # Adds synchronous behavior to the testing
    # rospy.sleep(1)
    # my_node.command_callback("Dance")
    
    my_node.loop()
    #rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.

