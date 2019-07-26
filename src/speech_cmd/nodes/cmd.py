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
            if msg.data == "turn left ninety":
                # ask to confirm operation
                self.engine.say('Confirm left turn ninety.')
                self.engine.iterate()
                # set status accordingly
                self.status = "await_confirm_left_ninety"
            if msg.data == "turn right ninety":
                self.engine.say('Confirm right turn ninety.')
                self.engine.iterate()
                self.status = "await_confirm_right_ninety"
            if msg.data == "turn left sixty":
                self.engine.say('Confirm left turn sixty.')
                self.engine.iterate()
                self.status = "await_confirm_left_sixty"
            if msg.data == "turn right sixty":
                self.engine.say('Confirm right turn sixty.')
                self.engine.iterate()
                self.status = "await_confirm_right_sixty"
            if msg.data == "turn left thirty":
                self.engine.say('Confirm left turn thirty.')
                self.engine.iterate()
                self.status = "await_confirm_left_thirty"
            if msg.data == "turn right thirty":
                self.engine.say('Confirm right turn thirty.')
                self.engine.iterate()
                self.status = "await_confirm_right_thirty"
            if msg.data == "forward one meter":
                self.engine.say('Confirm forward one.')
                self.engine.iterate()
                self.status = "await_confirm_one"
            if msg.data == "forward two meters":
                self.engine.say('Confirm forward two.')
                self.engine.iterate()
                self.status = "await_confirm_two"
            if msg.data == "forward three meters":
                self.engine.say('Confirm forward three.')
                self.engine.iterate()
                self.status = "await_confirm_three"
            if msg.data == "forward four meters":
                self.engine.say('Confirm forward four.')
                self.engine.iterate()
                self.status = "await_confirm_four"
            if msg.data == "forward five meters":
                self.engine.say('Confirm forward five.')
                self.engine.iterate()
                self.status = "await_confirm_five"
            if msg.data == "forward six meters":
                self.engine.say('Confirm forward six.')
                self.engine.iterate()
                self.status = "await_confirm_six"
            if msg.data == "backward one meter":
                self.engine.say('Confirm backward one.')
                self.engine.iterate()
                self.status = "await_confirm_backward_one"
            if msg.data == "backward two meters":
                self.engine.say('Confirm backward two.')
                self.engine.iterate()
                self.status = "await_confirm_backward_two"
            if msg.data == "backward three meters":
                self.engine.say('Confirm backward three.')
                self.engine.iterate()
                self.status = "await_confirm_backward_three"
            if msg.data == "backward four meters":
                self.engine.say('Confirm backward four.')
                self.engine.iterate()
                self.status = "await_confirm_backward_four"
            if msg.data == "backward five meters":
                self.engine.say('Confirm backward five.')
                self.engine.iterate()
                self.status = "await_confirm_backward_five"
            if msg.data == "backward six meters":
                self.engine.say('Confirm backward six.')
                self.engine.iterate()
                self.status = "await_confirm_backward_six"
            if msg.data == "feedback battery":
                # status feedback, doesn't need confirmation
                # dummy value, needs fitting topic subscription
                print("Battery Status: 78%.")
                self.engine.say('Battery Status is 78%.')
                self.engine.iterate()
        else:
            # if confirmation is awaited
            if self.status == "await_confirm_left_ninety":
                # check for correct confirmation
                if msg.data == "confirm":
                    print("turn left.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 1.570796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_left_sixty":
                # check for correct confirmation
                if msg.data == "confirm":
                    print("turn left.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 1.070796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_left_thirty":
                # check for correct confirmation
                if msg.data == "confirm":
                    print("turn left.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0.570796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_right_ninety":
                if msg.data == "confirm":
                    print("turn right.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = -1.570796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_right_sixty":
                if msg.data == "confirm":
                    print("turn right.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = -1.070796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_right_thirty":
                if msg.data == "confirm":
                    print("turn right.")
                    # create Twist msg and publish
                    self.msg.linear.x = 0
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = -0.570796
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_one":
                if msg.data == "confirm":
                    print("forward one meter.")
                    self.msg.linear.x = 1
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_two":
                if msg.data == "confirm":
                    print("forward one meter.")
                    self.msg.linear.x = 2
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_three":
                if msg.data == "confirm":
                    print("forward three meters.")
                    self.msg.linear.x = 3
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_four":
                if msg.data == "confirm":
                    print("forward four meter.")
                    self.msg.linear.x = 4
                    self.msg.linear.y = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_five":
                if msg.data == "confirm":
                    print("forward five meter.")
                    self.msg.linear.x = 5
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_six":
                if msg.data == "confirm":
                    print("forward six meter.")
                    self.msg.linear.x = 6
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_backward_one":
                if msg.data == "confirm":
                    print("backward one meter.")
                    self.msg.linear.x = -1
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_backward_two":
                if msg.data == "confirm":
                    print("backward two meters.")
                    self.msg.linear.x = -2
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_backward_three":
                if msg.data == "confirm":
                    print("backward three meters.")
                    self.msg.linear.x = -3
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_backward_four":
                if msg.data == "confirm":
                    print("backward four meters.")
                    self.msg.linear.x = -4
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_backward_five":
                if msg.data == "confirm":
                    print("backward five meters.")
                    self.msg.linear.x = -5
                    self.msg.linear.y = 0
                    self.msg.linear.z = 0
                    self.msg.angular.z = 0
                    self.pub.publish(self.msg)
            if self.status == "await_confirm_backward_six":
                if msg.data == "confirm":
                    print("backward six meters.")
                    self.msg.linear.x = -6
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
