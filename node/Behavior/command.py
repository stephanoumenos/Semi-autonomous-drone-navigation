import rospy
from st5drone.msg import BehaviorStatus

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

    def command_callback(self, msg):
        for behavior in self.behaviors:
            message = BehaviorStatus()
            message.behavior_name = behavior
            message.active = False

            self.behavior.publish(message)

        for call in self.commands[msg]:
            self.event_queue.enter(call[0], 0, self.contact_behavior, (call[1]))

    def contact_behavior(self, behavior_name):
        message = BehaviorStatus()
        message.behavior_name = behavior_name
        message.active = True

        self.behavior.publish(message)

