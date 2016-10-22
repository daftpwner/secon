#!/usr/bin/env python

'''
This node acts as the state machine controller for Brain and the entire
robotic system.

It additionally uses dynamic reconfigure to change and adjust ROS and Arduino
parameters during operation, allowing rapid development.

'''
from threading import Lock

import rospy
from std_msgs.msg import Header

from dynamic_reconfigure.server import Server
from secon2017_ros.cfg import BrainStateMachinConfig as BSMCfg
from secon2017_ros.msg import BrainState, BrainParameters, BrainCommand


class BrainStateMachine():

    def __init__(self):

        # Initialize node
        rospy.init_node("brain_state_machine")
        # Tracks current state
        self.lock = Lock()
        self.new_info = True
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
        self.parameter_pub = rospy.Publisher(
            "brain_parameters", BrainParameters
        )
        self.command_pub = rospy.Publisher(
            "brain_commands", BrainCommand
        )
        # Start state machine loop
        self.run()

    def run(self):
        rate = rospy.rate(self.rate)
        while not rospy.is_shutdown():
            self.generate_state_commands()
            rate.sleep()

    def generate_state_commands(self):
        # Threadsafe stuff
        with self.lock:
            if not self.new_info:
                return
            else:
                state = self.state
                self.new_info = False

        cmd_msg = BrainCommand()
        cmd_msg.header = Header()

        # Sends commands based on current state
        if self.states[self.current_state] == "wait_for_start":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "start":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG1_wall":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG1":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "align_to_STG1":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "perform_STG1":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG3_wall":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG3":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "align_to_STG3":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "perform_STG3":
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "end":
            self.command_pub.publish(cmd_msg)
            return

    def reconfigure_callback(self, config, level):
        # Reassigns and resends parameters to Arduino's
        msg = BrainParameters()
        msg.header = Header()

        msg.fl_pid = \
            [
                config["front_left_kp"],
                config["front_left_ki"],
                config["front_left_kd"]
            ]

        msg.fr_pid = \
            [
                config["front_right_kp"],
                config["front_right_ki"],
                config["front_right_kd"]
            ]

        msg.bl_pid = \
            [
                config["back_left_kp"],
                config["back_left_ki"],
                config["back_left_kd"],
            ]

        msg.br_pid = \
            [
                config["back_right_kp"],
                config["back_right_ki"],
                config["back_right_kd"],
            ]
        self.parameter_pub.publish(msg)
        return

    def brain_state_callback(self, brain_state):
        # Updates info dictionary of robot state information
        # Trips a flag for new info
        with self.lock:
            self.state = brain_state
            self.new_info = True
        return
