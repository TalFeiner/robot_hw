#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
from serial.tools import list_ports
from wheel_velocity_control import wheelVelocity

global ser, ser2, wheel_vel_calc


def open_serial_port():
    global ser, ser2
    baud = ""
    try:
        baud = rospy.get_param('~baud')
    except rospy.ROSException as e:
        rospy.logerr("ROSException: " + e)
        rospy.logerr("could not get baud param value, trying default value")
        pass

    port = ""
    try:
        port = rospy.get_param('~port')
    except rospy.ROSException as e:
        rospy.logerr("ROSException: " + e)
        rospy.logerr("could not get port param name, trying default port")
        pass

    if not baud:
        baud = 115200

    if not port:
        ports = list_ports.comports()
        port = []
        for p in ports:
            port.append(p.device)

        if len(ports) == 1 and ("/dev/ttyUSB" in port[0]):
            ser = serial.Serial(port[0], baud, write_timeout=0.5)  # open serial port
            # print(ser.name)         # check which port was really used
            port = port[0]
        else:
            rospy.logerr("Error port is not found")

    else:
        ser = serial.Serial(port, baud, write_timeout=0.5)

    ser2 = serial.Serial("/dev/ttyUSB1", baud)


def cmd_vel_cb(vel):
    global ser, wheel_vel_calc
    cmd_angular_left, cmd_angular_right = wheel_vel_calc.cmd_vel2angular_wheel_velocity(vel)
    cmd_angular_left = int(cmd_angular_left*500)
    cmd_angular_right = int(cmd_angular_right*500)
    send = str(str(cmd_angular_left) + str(";") + str(cmd_angular_right) + '\n')
    ser.write(bytes(send, encoding='utf8'))
    print("sending: " + str(send))
    rospy.sleep(0.001)
    ser.flushOutput()


rospy.init_node("blattoidea_hw_node", anonymous=True)
open_serial_port()
wheel_vel_calc = wheelVelocity()
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)
while not rospy.is_shutdown():
    if(ser2.in_waiting > 0):
        line = ser2.readline()
        print("msg - ", line)
    rospy.sleep(0.005)
    ser.flushInput()
