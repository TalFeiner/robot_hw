#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
from serial.tools import list_ports

global ser, ser2


def cmd_vel2angular_wheel_velocity(vel, diameter=0.1651, wheelsSeparation=0.42):
    cmd_linear_left = vel.linear.x + ((vel.angular.z * wheelsSeparation) / 2.0)
    cmd_linear_right = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2.0)
    cmd_angular_left = cmd_linear_left / (diameter / 2)
    cmd_angular_right = cmd_linear_right / (diameter / 2)
    # print("cmd_angular_left: ", cmd_angular_left, "cmd_angular_right: ", cmd_angular_right)
    return cmd_angular_left, cmd_angular_right


def open_serial_port():
    global ser, ser2
    baud = ""
    baud2 = ""
    try:
        baud = rospy.get_param('~baud', 115200)
    except rospy.ROSException as e:
        rospy.logerr("ROSException: " + e)
        rospy.logerr("could not get baud param value, trying default value")
        pass
    try:
        baud2 = rospy.get_param('~baud2', 115200)
    except rospy.ROSException as e:
        rospy.logerr("ROSException: " + e)
        rospy.logerr("could not get baud2 param value, trying default value")
        pass

    port = ""
    port2 = ""
    try:
        port = rospy.get_param('~port')
    except rospy.ROSException as e:
        rospy.logerr("ROSException: " + e)
        rospy.logerr("could not get port param name, trying default port")
        pass
    try:
        port2 = rospy.get_param('~port2')
    except rospy.ROSException as e:
        rospy.logerr("ROSException: " + e)
        rospy.logerr("could not get port param name, trying default port")
        pass

    if not baud:
        baud = 115200
    if not baud2:
        baud2 = 115200

    if not port:
        ports = list_ports.comports()
        port = []
        for p in ports:
            port.append(p.device)

        if len(ports) == 2 and ("/dev/ttyUSB" in port[0]) and ("/dev/ttyUSB" in port[1]):
            ser = serial.Serial(port[0], baud, write_timeout=0.5, timeout=0.5)  # open serial port
            # print(ser.name)         # check which port was really used
        else:
            rospy.logerr("Error port is not found")

    else:
        ser = serial.Serial(port, baud, write_timeout=0.5, timeout=0.5)

    if not port2:
        ports = list_ports.comports()
        port = []
        for p in ports:
            port.append(p.device)

        if len(ports) == 2 and ("/dev/ttyUSB" in port[0]) and ("/dev/ttyUSB" in port[1]):
            ser2 = serial.Serial(port[1], baud, write_timeout=0.5, timeout=0.5)  # open serial port
            # print(ser.name)         # check which port was really used
        else:
            rospy.logerr("Error port is not found")

    else:
        ser2 = serial.Serial(port2, baud2, write_timeout=0.5, timeout=0.5)  # open serial port

    ser.flushInput()
    ser2.flushInput()
    ser.flushOutput()
    ser2.flushOutput()


def cmd_vel_cb(vel):
    global ser
    cmd_angular_left, cmd_angular_right = cmd_vel2angular_wheel_velocity(vel)
    cmd_angular_left = int(cmd_angular_left*500)
    cmd_angular_right = int(cmd_angular_right*500)
    send = str(str(cmd_angular_left) + str(";") + str(cmd_angular_right) + '\n')
    ser.write(bytes(send, encoding='utf8'))
    print("sending: " + str(send))
    ser.flushOutput()


rospy.init_node("blattoidea_hw_node", anonymous=True)
open_serial_port()
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)
while not rospy.is_shutdown():
    if(ser2.in_waiting > 0):
        line = ser2.readline()
        ser2.flushInput()
        print("msg - ", line)
    rospy.sleep(0.005)
