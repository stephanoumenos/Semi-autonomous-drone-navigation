
import rospy
from behavior import Behavior

from std_msgs.msg import Empty

class TakeOff(Behavior):

    def __init__(self):
        super(TakeOff, self).__init__()
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=0)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=0)

    def on_status_on(self):
        self.active = True
        self.pub_takeoff.publish(Empty())
        rospy.sleep(2)
        self.pub_land.publish(Empty())
        self.active = False


    def on_status_off(self):
        pass


