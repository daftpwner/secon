#!/bin/env python

'''
This node handles serial communications with both Arduino devices of the robotic
system.  Commands and parameter updates are forwarded from the
Brain State Machine and passed onto the appropriate Arduino.

Sensor data is also forwarded up from the Arduino to the State Machine.
'''
