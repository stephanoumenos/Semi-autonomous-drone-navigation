#!/usr/bin/env python
import rospy
import time

class EventQueue():

    def __init__(self):

        # Sets up a list of events as such that each element represents an event that should run at a delay
        # [x, event]
        self.listEvents = []

    def setEvents(self, list_events):
        timeOnCreation = rospy.Time.now()

        self.listEvents = map(list, list_events)

        for element in self.listEvents:
            # print(element[0], timeOnCreation.to_sec())
            element[0] += timeOnCreation.to_sec()
    
    def returnValidEvents(self):
        result = []
        timeOnFunctionCall = rospy.Time.now().to_sec()

        for element in self.listEvents:
            if timeOnFunctionCall > element[0]:
                result.append(element)
                self.listEvents.remove(element)

        return result
    
    def __sizeof__(self):
        return len(self.listEvents)    
