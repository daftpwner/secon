#!/usr/bin/env python

'''
This node acts as the state machine controller for Brain and the entire
robotic system.

It additionally uses dynamic reconfigure to change and adjust ROS and Arduino
parameters during operation, allowing rapid development.

'''

import rospy

from dynamic_reconfigure.server import Server
from secon2017_ros.cfg import BrainStateMachinConfig as BSMCfg
from secon2017_ros.msg import BrainState

class BrainStateMachine():

    def __init__(self):

        # Initialize node
        rospy.init_node("brain_state_machine")
        # Tracks current state
        self.current_state = 0
        # Lists all states of Brain
        self.states = \
        [
            "wait_for_start",
            "start",

            "nav_to_STG1_wall",
            "nav_to_STG1",
            "align_to_STG1",
            "perform_STG1",

            "nav_to_STG3_wall",
            "nav_to_STG3",
            "align_to_STG3",
            "perform_STG3",

            "end"
        ]

        # Load parameter defaults here

        # Start dynamic reconfigure server
        self.dynamic_server = Server(BSMCfg, self.reconfigure_callback)

        # Set subscribers
        rospy.Subscriber("brain_state", BrainState, self.brain_state_callback)

        # Set publishers

        # Start state machine loop
        self.run()

    def run(self):
        rate = rospy.rate(self.rate)
        while not rospy.is_shutdown():
            self.generate_state_commands()
            rate.sleep()

    def generate_state_commands(self):
        # Sends commands based on current state
        if self.states[self.current_state] == "wait_for_start":
            return
        elif self.states[self.current_state] == "start":
            return
        elif self.states[self.current_state] == "nav_to_STG1_wall":
            return
        elif self.states[self.current_state] == "nav_to_STG1":
            return
        elif self.states[self.current_state] == "align_to_STG1":
            return
        elif self.states[self.current_state] == "perform_STG1":
            return
        elif self.states[self.current_state] == "nav_to_STG3_wall":
            return
        elif self.states[self.current_state] == "nav_to_STG3":
            return
        elif self.states[self.current_state] == "align_to_STG3":
            return
        elif self.states[self.current_state] == "perform_STG3":
            return
        elif self.states[self.current_state] == "end":
            return

    def reconfigure_callback(self, config, level):
        # Reassigns and resends parameters to Arduino's
        return

    def brain_state_callback(self, brain_state):
        # Updates info dictionary of robot state information
        # Trips a flag for new info
        return
