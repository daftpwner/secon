#!/usr/bin/env python

'''
This node handles serial communications with both Arduino devices of the
system.  Commands and parameter updates are forwarded from the
Brain State Machine and passed onto the appropriate Arduino.

Sensor data is also forwarded up from the Arduino to the State Machine.
'''

import rospy
from secon2017_ros.msg import BrainState, BrainCommand
from std_msgs.msg import Header

import serial
import re
from threading import Lock


class EmbeddedInterface():

    def __init__(self):
        # Initialize node
        rospy.init_node("embedded_interface")
        self.pinky_connected = False  # Initialize to True when Pinky present
        self.brain_port_lock = Lock()

        self.prev_msg = BrainState()
        self.prev_msg.header = Header()
        self.prev_msg.header.stamp = rospy.Time.now()
        self.ros_msg = self.prev_msg

        # regex strings to extract data from strings
        # [[01];]+ parses out arbitrary number of 1/0's
        # [[+-]\d+.\d+;]+ should parse out an arbitrary number of floats
        # with signs and delimited by ';'
        self.brain_regex_dict = {
            "message": re.compile(
                "B:sw:[01]*wv:(-?\d+\.\d+;?){4}sq:\d+rt:\d+\n"
            ),
            "switches": re.compile("sw:[01]*"),
            "wvel": re.compile("wv:(-?\d+\.\d+;?){4}"),
            "sequence": re.compile("sq:\d+"),
            "rotation_sequence": re.compile("rt:\d+\n")
        }

        # self.pinky_regex = "P:"
        # command string template
        # x,y,angular velocities; stage 1, 3 triggers
        self.command_string = "C{fl_wvel:=+05d},{fr_wvel:=+05d},{bl_wvel:=+05d},{br_wvel:=+05d};{STG1},{STG3}\n"
        # parameter string template
        self.parameter_string = "P{fl_kp:=+05.3f},{fl_kp:=+05.3f},{fl_kp:=+05.3f},{fr_kp:=+05.3f},{fr_ki:=+05.3f},{fr_kd:=+05.3f},{bl_kp:=+05.3f},{bl_ki:=+05.3f},{bl_kd:=+05.3f},{br_kp:=+05.3f},{br_ki:=+05.3f},{br_kd:=+05.3f}\n"
        # Set parameters
        self.rate = rospy.get_param("update_frequency", 10)
        self.brain_port = rospy.get_param(
            "~brain_serial_port", "/dev/arduinos/brain")
        self.brain_baud = rospy.get_param(
            "~brain_baud_rate", "9600")
        self.pinky_port = rospy.get_param(
            "~pinky_serial_port", "/dev/arduinos/pinky")
        # Set callbacks
        self.brain_cmd_sub = rospy.Subscriber("brain_cmd", BrainCommand, self.command_callback)
        # Set publishers
        self.brain_state_pub = rospy.Publisher("brain_state", BrainState, queue_size=2)
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
                "fr_wvel": command.wheel_vels[1],
                "bl_wvel": command.wheel_vels[2],
                "br_wvel": command.wheel_vels[3],
                "STG1": int(command.stg1),
                "STG3": int(command.stg3)
            }

        with self.brain_port_lock:
            self.Brain.write(self.command_string.format(**cmd_dict))
            rospy.loginfo(self.command_string.format(**cmd_dict))
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
            self.brain_port.write(self.parameter_string.format(**param_dict))
        return

    def run(self):
        # Main loop to read from serial ports
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            # Blocks while reading
            with self.brain_port_lock:
                brain_data = self.Brain.readline()
            # Don't bother once disconnected from Pinky
            # if self.pinky_connected:
            #    pinky_data = self.Pinky.readline()
            # Parse the command
            self.parse_state_string(brain_data)
            rospy.loginfo("Read data: %s" %brain_data)
            rate.sleep()
        return

    def parse_state_string(self, state_str):
        # Parses raw serial string into data and publishes it
        messages = self.brain_regex_dict["message"].search(state_str)
        msg = messages.group() if messages else []
        if msg == []:
            return

        self.ros_msg = BrainState()
        self.ros_msg.header = Header()
        self.ros_msg.header.stamp = rospy.Time.now()
        sw_msg = self.brain_regex_dict["switches"].search(msg).group(0)
        wvel_msg = self.brain_regex_dict["wvel"].search(msg).group(0)
        seq_msg = self.brain_regex_dict["sequence"].search(msg).group(0)
        rot_msg = self.brain_regex_dict["rotation_sequence"].search(msg).group(0)

        self.ros_msg.switches = [bool(int(s)) for s in sw_msg[3:]]
        self.ros_msg.wheel_vels = [float(vel) for vel in wvel_msg[3:].split(";")]
        self.ros_msg.sequence = "{:05d}".format(int(seq_msg[3:]))
        self.ros_msg.rotation_sequence = "{:05d}".format(int(rot_msg[3:]))

        self.integrate_odometry()
        self.brain_state_pub.publish(self.ros_msg)
        return

    def integrate_odometry(self):

        self.prev_msg = self.ros_msg
        return


if __name__ == "__main__":
    node = EmbeddedInterface()
