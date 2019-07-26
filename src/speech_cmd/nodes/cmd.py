#!/usr/bin/env python

import roslib; roslib.load_manifest('speech_cmd')
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import *
import actionlib
from move_base_msgs.msg import *

import os
import commands
import pyttsx

class cmd(object):

    def __init__(self):
        # Start node
        rospy.init_node("cmd")


        # Configure ROS settings
        self.started = False
        self.status = "new_cmd"
        self.engine = pyttsx.init()
        self.engine.startLoop(False)

        # create Twist Message
        self.msg = Twist()
        # publish to TurtelSim for demonstration
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist)

        # subscribe to pocketsphinx output
        rospy.Subscriber('recognizer/output', String, self.voice_cmd_map)

        rospy.Service("~start", Empty, self.start)

        self.start_cmd()


    def start_cmd(self):
        rospy.loginfo("Starting speech command interface... ")
        # Voice Output Settings
        rate = self.engine.getProperty('rate')
        self.engine.setProperty('rate', rate-50)
        volume = self.engine.getProperty('volume')
        self.engine.setProperty('volume', volume+0.1)
        voices = self.engine.getProperty('voices')

        self.started = True

        # keep node from exiting
        rospy.spin()


    def stop_cmd(self):
        if self.started:
            self.engine.endLoop()
            self.started = False


    def start(self, req):
        self.start_cmd()
        rospy.loginfo("speech_cmd started")
        return EmptyResponse()


    def stop(self, req):
        self.stop_cmd()
        rospy.loginfo("speech_cmd stopped")
        return EmptyResponse()


    def voice_cmd_map(self, msg):
        rospy.loginfo(msg.data)
        # listen for new command
        if self.status == "new_cmd":
            # check for trigger words
            if msg.data == "move left":
                # ask to confirm operation
                self.engine.say('Please confirm left rotate.')
                self.engine.iterate()
                # set status accordingly
                self.status = "await_confirm_left"
            if msg.data == "move right":
                self.engine.say('Please confirm right rotate.')
                self.engine.iterate()
                self.status = "await_confirm_right"
            if msg.data == "move forward one meter":
                self.engine.say('Please confirm forward.')
                self.engine.iterate()
                self.status = "await_confirm_one"
            if msg.data == "move forward three meter":
                self.engine.say('Please confirm forward.')
                self.engine.iterate()
                self.status = "await_confirm_three"
            if msg.data == "feedback battery":
                # status feedback, doesn't need confirmation
                # dummy value, needs fitting topic subscription
                print("Battery Status: 78%.")
                self.engine.say('Battery Status is 78%.')
                self.engine.iterate()
        else:
            # if confirmation is awaited
            if self.status == "await_confirm_left":
                # check for correct confirmation
                if msg.data == "confirm":
                    print("rotate left.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 1.570796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_right":
                if msg.data == "confirm":
                    print("rotate right.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = -1.570796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_one":
                if msg.data == "confirm":
                    print("drive forward.")
                    self.msg.linear.x = 1
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_three":
                if msg.data == "confirm":
                    print("drive forward.")
                    self.msg.linear.x = 3
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)

            # reset status
            self.status = "new_cmd"



if __name__ == "__main__":
    try:
        start = cmd()
    except KeyboardInterrupt:
        sys.exit(1)
