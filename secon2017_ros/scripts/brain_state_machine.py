#!/usr/bin/env python

'''
This node acts as the state machine controller for Brain and the entire
robotic system.

It additionally uses dynamic reconfigure to change and adjust ROS and Arduino
parameters during operation, allowing rapid development.

NOTE: The forward face of the robot is defined as the side
        containing the Stage 1 interface component.

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
        self.wheel_coords = \
            {
                "front_left":rospy.get_param("front_left_wheel_coords",[1 -1]),
                "front_right":rospy.get_param("front_right_wheel_coords",[1 -1]),
                "back_left":rospy.get_param("back_left_wheel_coords",[1 -1]),
                "back_right":rospy.get_param("back_right_wheel_coords",[1 -1]),                
            }

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
            # No updates if no new data
            if not self.new_info:
                return
            else:
                # state contains all state variables
                state = self.state
                self.new_info = False

        # message containing Brain commands
        # "cmd_msg" contents:
        # cmd_msg.header: ROS Header with timestamp
        # cmd_msg.wheel_vels: commanded wheel velocities
        # 0: Front Left wheel
        # 1: Front Right wheel
        # 2: Back Left wheel
        # 3: Back Right wheel
        # cmd_msg.stg1: Stage 1 Trigger
        # cmd_msg.stg3: Stage 3 Trigger
        cmd_msg = BrainCommand()
        cmd_msg.header = Header()

        # "state" contents:
        # state.header: ROS Header with timestamp
        # state.switches: list of boolean switch states
        # 0: Start Switch
        # 1: Stage 1 Contact Switch
        # 2: Stage 3 Contact Switch
        # 3:
        # state.wheel_vels: list of wheel velocities
        # 0: Front Left wheel
        # 1: Front Right wheel
        # 2: Back Left wheel
        # 3: Back Right wheel
        # state.odom: ROS Odometry message containing position and velocity
        # state.sequence: string containing decoded sequence, "00000" default
        # state.rotation_sequence: string of entered sequence, "00000" default

        # Sends commands based on current state
        if self.states[self.current_state] == "wait_for_start":
            # wait for start button to be pressed
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "start":
            # Initialize and start Pinky and wait for clearance
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG1_wall":
            # Navigate to the wall
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG1":
            # Navigate along wall to Stage 1
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "align_to_STG1":
            # Final alignment and connection to Stage 1
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "perform_STG1":
            # Trigger Stage 1
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG3_wall":
            # Navigate to Stage 3 wall
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "nav_to_STG3":
            # Navigate along wall to Stage 3
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "align_to_STG3":
            # Final alignment and connection to Stage 3
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "perform_STG3":
            # Trigger Stage 3
            self.command_pub.publish(cmd_msg)
            return

        elif self.states[self.current_state] == "end":
            # Finished, cease operations
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

    # Inputs:
    #   xvel: float representing x velocity in mm/s
    #   yvel: float representing y velocity in mm/s
    #   ang_vel: float representing anglular velocity in rad/s
    # Output:
    #   tuple containing the four wheel velocities in mm/s
    def mix_wheel_velocities(self, x_vel, y_vel, ang_vel):
        # Convert linear and angular velocities to wheel velocities

        return (fl_wvel, fr_wvel, bl_wvel, br_wvel)
