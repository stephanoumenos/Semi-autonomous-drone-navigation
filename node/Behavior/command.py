#!/usr/bin/env python

import rospy
from st5drone.msg import BehaviorStatus
from std_msgs.msg import String

import sched, time

class Command:

    def __init__(self):
        self.behaviors = ['Hover', 'MoveLeft', 'MoveUp', 'MoveRight', 'RotateLeft']
        self.commands  = {
            'Stop' : [(0, 'Hover')],
            'Dance' : [(0, 'MoveLeft'), (0, 'MoveUp'), (2.5, 'MoveRight'), (3.2, 'Hover')]
        }

        self.command = rospy.Subscriber("/command", String, self.command_callback, queue_size=1)
        self.behavior = rospy.Publisher("/behavior", BehaviorStatus, queue_size=1)

        self.event_queue = sched.scheduler(time.time, time.sleep)
        self.event_queue.run()

    def command_callback(self, msg):
        print("Entered callback")
        for behavior in self.behaviors:
            message = BehaviorStatus()
            message.behavior_name = behavior
            message.active = False
            # print(message)
            self.behavior.publish(message)
        
        
        event_queue = sched.scheduler(time.time, time.sleep)
        for call in self.commands[msg]:
            # print("Requested call " + call[1] + "for a time " + str(call[0])   )  
            # print(call[1])
           
            event_queue.enter(call[0], 0, self.contact_behavior, (call[1],) )

        event_queue.run()

    def contact_behavior(self, behavior_name):
    
        message = BehaviorStatus()
        message.behavior_name = behavior_name
        message.active = True

        # print(message)

        self.behavior.publish(message)


if __name__ == '__main__':     # This is the main thread, thread #1
    rospy.init_node('Command')

    my_node = Command()

    # rospy.sleep(1)
    # my_node.command_callback("Dance")
    
    #my_node.loop()
    rospy.spin() # useless... since loop already blocks. If you have
                 # no idle job (i.e. loop) to do outside event
                 # handling, rospy.spin() is mandatory in order to
                 # prevent from immediate termination of your node.

