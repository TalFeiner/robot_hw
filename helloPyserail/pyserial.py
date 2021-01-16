#!/usr/bin/env python3

import rospy
import serial
from serial.tools import list_ports

rospy.init_node("pyserial")

baud = ""
try:
    baud = rospy.get_param('~baud')
except:
    rospy.logerr("could not get baud param value, trying default value")
    pass

port = ""
try:
    port = rospy.get_param('~port')
except:
    rospy.logerr("could not get port param name, trying default port")
    pass

if not baud:
    baud = 115200

if not port:
    ports = list_ports.comports()
    port = []
    for p in ports:
        port.append(p.device)
    print("port list: " , port)
    print(len(ports), 'ports found')

    if len(ports) is 1 and (port[0] == "/dev/ttyUSB0" or port[0] == "/dev/ttyUSB1"):
        ser = serial.Serial(port[0], 115200)  # open serial port
        print(ser.name)         # check which port was really used
    else:
        rospy.logerr("Error port is not found")

else:
    ser = serial.Serial(port, 115200)
    print(ser.name)

c=0.01
while not rospy.is_shutdown():
    # line = ser.readline()
    # print("line: ", line)
    # var = line.split(b";")
    # print ("var[0]: " + str(var[0]) + ", var[1]: " , float(var[1]))
    # a = float(var[1])
    # print(a, type(a))
    send = str(str("Hello;") + str(c))
    ser.write(bytes(send, encoding='utf8'))

    line = ser.readline()
    print("line: ", line)
    c += 0.01
    rospy.sleep(0.5)
