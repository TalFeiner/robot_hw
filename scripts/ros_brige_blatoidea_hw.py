#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from  std_srvs.srv import Empty, EmptyResponse
import serial
from serial.tools import list_ports
import numpy as np
# import string

global time_old, Kp, Ki, Kd, leftI, leftErrorOld, rightErrorOld, dist, theta, D, wheelsSeparation
Kp = 45.0; Ki = 13.5; Kd = 4.5
leftI = 0.0; rightI = 0.0
leftErrorOld = 0.0; rightErrorOld = 0.0
dist = 0; theta = 0
D = 0.1651; wheelsSeparation = 0.42     #   [m]

rospy.init_node("blattoidea_hw_node", anonymous=True)

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
    # print("port list: " , port)
    # print(len(ports), 'ports found')

    if len(ports) is 1 and ("/dev/ttyUSB" in port[0]):
        ser = serial.Serial(port[0], baud)  # open serial port
        # print(ser.name)         # check which port was really used
    else:
        rospy.logerr("Error port is not found")

else:
    ser = serial.Serial(port, baud)
    # print(ser.name)

pub_left_vel = rospy.Publisher("/blattoidea/cmd_left", Int8, queue_size = 1)
pub_right_vel = rospy.Publisher("/blattoidea/cmd_right", Int8, queue_size = 1)
time_old = rospy.Time.now()

def cmd_vel_cb (vel):
    global time_old, Kp, Ki, Kd, leftI, rightI, leftErrorOld, rightErrorOld, dist, theta, D, wheelsSeparation

    if (vel.linear.x is 0.0 and vel.angular.z is 0.0):
        msg_left = Int8()
        msg_right = Int8()
        msg_left.data = np.int8(0)
        msg_right.data = np.int8(0)
        pub_left_vel.publish(msg_left)
        pub_right_vel.publish(msg_right)

    else:
        line = ser.readline()
        # try:
        #     line = ''.join(filter(lambda c: c in string.printable, line))
        # except:
        #     pass
        print("line: ", line)
        var = line.split(b";")
        if (len(var) > 2 and not b"Arduino exceptions" in var[-1]):
            vel_left = float(var[1])
            vel_right = float(var[2])
            print ("vel_left: ", vel_left, " vel_right: ", vel_right)

            # linearVelL = (D / 2) * vel_left     #   [m/sec]
            # linearVelR = (D / 2) * vel_right     #   [m/sec]
            # omega = (linearVelR - linearVelL) / wheelsSeparation        #   [rad/sec]
            # linearV = (linearVelR + linearVelL) / 2     #   [m/sec]

            # dist = linearV * dt + dist      #   [m]
            # theta = omega * dt + theta      #   [rad]
            # x = dist * cos(theta)       #   [m]
            # y = dist * sin(theta)       #   [m]

            # linearError = vel.linear.x - linearVel
            # angularError = vel.angular.z - angularVel
            
            cmd_linear_left = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2) 
            cmd_linear_right = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2)
            cmd_angular_left = cmd_linear_left / (D / 2)
            cmd_angular_right = cmd_linear_right / (D / 2)  

            error_left = vel_left - cmd_angular_left
            error_right = vel_right - cmd_angular_right
            dt = (time_old - rospy.Time.now()).to_sec()
            time_old = rospy.Time.now()
            leftI += error_left * dt
            rightI += error_right * dt
            leftD = (error_left - leftErrorOld) / dt
            rightD = (error_right - rightErrorOld) / dt
            leftErrorOld = error_left
            rightErrorOld = error_right

            left_pid = Kp * error_left + Ki * leftI + Kd * leftD
            right_pid = Kp * error_right + Ki * rightI + Kd * rightD
            
            if (left_pid > 127):
                left_pid = 127
            if (left_pid < -127):
                left_pid = -127

            if (right_pid > 127):
                right_pid = 127
            if (right_pid < -127):
                right_pid = -127

            msg_left = Int8()
            msg_right = Int8()

            msg_left.data = np.int8(left_pid)
            msg_right.data = np.int8(right_pid)
            pub_left_vel.publish(msg_left)
            pub_right_vel.publish(msg_right)
            
            print("msg_left: ", msg_left, " msg_right: ", msg_right)
        elif (not b"Arduino exceptions" in var[-1]):
            msg_left = Int8()
            msg_right = Int8()
            msg_left.data = np.int8(0)
            msg_right.data = np.int8(0)
            pub_left_vel.publish(msg_left)
            pub_right_vel.publish(msg_right)
            
            print("msg_left: ", msg_left, " msg_right: ", msg_right)

def reset_cov_cb (empty):
    send = str(str("true;") + str("null"))
    ser.write(bytes(send, encoding='utf8'))
    rospy.loginfo("dead reckoning covariance reset, done.")
    return EmptyResponse()

rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)
rospy.Service("/blattoidea/reset_dead_reckoning_cov", Empty, reset_cov_cb)

rospy.spin()
