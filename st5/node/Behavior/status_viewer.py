#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
from st5drone.msg import BehaviorStatus
import Tkinter as tk

class StatusViewer :

    def __init__(self):
        self.status = {}
        self.labels = {}
        self.sub_behavior = rospy.Subscriber("behavior_status", BehaviorStatus, self.on_behaviors, queue_size=10)
        self.pub_ping     = rospy.Publisher ("behavior",         BehaviorStatus,                    queue_size=10)

        self.ping = BehaviorStatus()
        self.ping.behavior_name = 'ping'


    def on_behaviors(self, msg):
        self.status[msg.behavior_name] = msg.active

    def on_timeout(self) :
        if rospy.is_shutdown():
            self.root.destroy()
            return
        self.status = {}
        self.pub_ping.publish(self.ping)
        rospy.sleep(0.4)
        for _, label in self.labels.items():
            label.configure(background = self.error_bg, foreground = self.error_fg)
        for name, value in self.status.items() :
            if value :
                bg_color = self.active_bg
                fg_color = self.active_fg
            else :
                bg_color = self.inactive_bg
                fg_color = self.inactive_fg
            if name not in self.labels:
                label = tk.Label(self.pad, text=name)
                label.configure(anchor='w', padx=5)
                label.pack(fill=tk.X)
                self.labels[name] = label
            else :
                label = self.labels[name]
            label.configure(background = bg_color, foreground = fg_color)
        self.root.after(100, self.on_timeout)

    def loop(self):
        self.root = tk.Tk()
        self.root.title('Behaviors')
        self.root.configure(relief='flat', padx=5, pady=5)

        title = tk.Label(self.root,text='Behaviors')
        title.pack()

        self.pad = tk.Frame(self.root)
        self.pad.configure(relief='sunken', borderwidth=2)
        self.pad.pack()

        self.active_bg = '#0000aa'
        self.active_fg = '#ccccff'
        self.inactive_bg = self.root.cget('bg')
        self.inactive_fg = '#0000aa'
        self.error_bg = '#aaaa00'
        self.error_fg = '#ff0000'

        self.root.after(1000, self.on_timeout)
        self.root.mainloop()


if __name__ == "__main__":

    rospy.init_node('status_viewer')

    status_viewer = StatusViewer()
    status_viewer.loop() # instead ot rospy.spin()

