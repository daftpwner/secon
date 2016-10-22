#!/usr/bin/env python

'''
This node handles serial communications with both Arduino devices of the
system.  Commands and parameter updates are forwarded from the
Brain State Machine and passed onto the appropriate Arduino.

Sensor data is also forwarded up from the Arduino to the State Machine.
'''

import rospy
from secon2017_ros.msg import BrainState

import serial
import regex as re
from threading import Lock

class EmbeddedInterface():

    def __init__(self):
        # Initialize node
        rospy.init_node("embedded_interface")
        self.pinky_connected = False  # Initialize to True when Pinky present
        self.brain_port_lock = Lock()
        # regex strings to extract data from strings
        # [[01];]+ parses out arbitrary number of 1/0's
        # [[+-]\d+.\d+;]+ should parse out an arbitrary number of floats
        # with signs and delimited by ';'
        self.brain_regex = \
            "B:sw:[[01];]+wvel:[[+-]\d+.\d+;]+seq:\d{5}rot:\d{5}"
        # self.pinky_regex = "P:"
        # command string template
        # x,y,angular velocities; stage 1, 3 triggers
        self.command_string = "C{fl_wvel:=+04d},{fr_wvel:=+04d},{bl_wvel:=+04d},{br_wvel:=+04d};{STG1},{STG3}\n"
        # parameter string template
        self.parameter_string = "P{fl_kp:=+05.3f},{fl_kp:=+05.3f},{fl_kp:=+05.3f},{fr_kp:=+05.3f},{fr_ki:=+05.3f},{fr_kd:=+05.3f},{bl_kp:=+05.3f},{bl_ki:=+05.3f},{bl_kd:=+05.3f},{br_kp:=+05.3f},{br_ki:=+05.3f},{br_kd:=+05.3f}\n"
        # Set parameters
        self.brain_port = rospy.get_param(
            "brain_serial_port", "/dev/arduinos/brain")
        self.pinky_port = rospy.get_param(
            "pinky_serial_port", "/dev/arduinos/pinky")
        # Set callbacks

        # Set publishers
        self.brain_state_pub = rospy.Publisher("brain_state", BrainState)
        # Open serial ports
        self.Brain = serial.Serial(
            self.brain_port, self.brain_baud, timeout=0.5)
        # self.Pinky = serial.Serial(
        #    self.pinky_port, self.pinky_baud, timeout=0.5)

        # Launch main listening loop
        self.run()

    def command_callback(self, command):
        # Forwards command operations
        cmd_dict = \
            {
                "fl_wvel": command.wheel_vels[0],
                "fl_wvel": command.wheel_vels[1],
                "fl_wvel": command.wheel_vels[2],
                "fl_wvel": command.wheel_vels[3],
                "STG1": command.stg1,
                "STG3": command.stg3
            }

        with self.brain_port_lock:
            self.brain_port.write(self.command_string.format(cmd_dict))
        return

    def parameter_callback(self, parameters):
        # Forwards parameter reassignments
        param_dict = \
            {
                "fl_kp": parameters.fl_pid[0],
                "fl_ki": parameters.fl_pid[1],
                "fl_kd": parameters.fl_pid[2],

                "fr_kp": parameters.fr_pid[0],
                "fr_ki": parameters.fr_pid[1],
                "fr_kd": parameters.fr_pid[2],

                "bl_kp": parameters.bl_pid[0],
                "bl_ki": parameters.bl_pid[1],
                "bl_kd": parameters.bl_pid[2],

                "br_kp": parameters.br_pid[0],
                "br_ki": parameters.br_pid[1],
                "br_kd": parameters.br_pid[2],
            }

        with self.brain_port_lock:
            self.brain_port.write(self.parameter_string.format(param_dict))
        return

    def run(self):
        # Main loop to read from serial ports
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            # Blocks while reading
            with self.brain_port_lock:
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
