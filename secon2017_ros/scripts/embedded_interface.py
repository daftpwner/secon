#!/usr/bin/env python

'''
This node handles serial communications with both Arduino devices of the robotic
system.  Commands and parameter updates are forwarded from the
Brain State Machine and passed onto the appropriate Arduino.

Sensor data is also forwarded up from the Arduino to the State Machine.
'''

import rospy
from secon2017_ros.msg import BrainState

import serial
import regex as re


class EmbeddedInterface():

    def __init__():
        # Initialize node
        rospy.init_node("embedded_interface")
        self.pinky_connected = False # Initialize to True when Pinky present
        # regex strings to extract data from strings
        self.brain_regex = "B:sw:*[]odom*[]wvel:*[]seq:*[]rot:*[]"
        self.pinky_regex = "P:"
        # command string template
        # x,y,angular velocities; stage 1, 3 triggers
        self.command_string = "C{xvel},{yvel},{zangv};{STG1},{STG3}"
        # parameter string template
        self.parameter_string = "P{}"
        # Wheel debug string
        self.debug_string = "W{w1vel},{w2vel},{w3vel},{w4vel}"
        # Set parameters
        self.brain_port = rospy.get_param("brain_serial_port", "/dev/arduinos/brain")
        self.pinky_port = rospy.get_param("pinky_serial_port", "/dev/arduinos/pinky")
        # Set callbacks

        # Set publishers
        self.brain_state_pub = rospy.Publisher("brain_state", BrainState)
        # Open serial ports
        self.Brain = serial.Serial(self.brain_port, self.brain_baud, timeout=0.5)
        self.Pinky = serial.Serial(self.pinky_port, self.pinky_baud, timeout=0.5)

        # Launch main listening loop
        self.run()

    def command_callback(self, command):
        # Forwards command operations
        return

    def parameter_callback(self, parameters):
        # Forwards parameter reassignments
        return

    def run(self):
        # Main loop to read from serial ports
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            brain_data = self.Brain.readline()
            # Don't bother once disconnected from Pinky
            if self.pinky_connected:
                pinky_data = self.Pinky.readline()
            # Parse the command
            self.parse_state_string(brain_data)

            self.parse_state_string(pinky_data)
            rate.sleep()
        return

    def parse_state_string(self, state_str):
        # Parses raw serial string into data and publishes it
        return
