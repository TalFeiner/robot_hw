#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_srvs.srv import Empty, EmptyResponse
import serial
from serial.tools import list_ports
import numpy as np
from wheel_velocity_control import PIDClass, wheelVelocity
# import string

rospy.init_node("blattoidea_hw_node", anonymous=True)

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
    # print("port list: " , port)
    # print(len(ports), 'ports found')

    if len(ports) == 1 and ("/dev/ttyUSB" in port[0]):
        ser = serial.Serial(port[0], baud)  # open serial port
        # print(ser.name)         # check which port was really used
    else:
        rospy.logerr("Error port is not found")

else:
    ser = serial.Serial(port, baud)
    # print(ser.name)


def cmd_vel_cb(vel, ser, pid_left, pid_right, wheel_vel, pub_left_vel, pub_right_vel):
    if(vel.linear.x == 0.0 and vel.angular.z == 0.0):
        msg_left = Int8()
        msg_right = Int8()
        msg_left.data = np.int8(0)
        msg_right.data = np.int8(0)
        pub_left_vel.publish(msg_left)
        pub_right_vel.publish(msg_right)
        pid_left.pid_reset(True)
        pid_right.pid_reset(True)

    else:
        line = ser.readline()
        # try:
        #     line = ''.join(filter(lambda c: c in string.printable, line))
        # except:
        #     pass
        print("line: ", line)
        var = line.split(b";")
        if (len(var) > 2 and b"Arduino exceptions" not in var[-1]):
            vel_left = float(var[1])
            vel_right = float(var[2])
            print("vel_left: ", vel_left, " vel_right: ", vel_right)

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
            # cmd_linear_left = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2) 
            # cmd_linear_right = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2)
            # cmd_angular_left = cmd_linear_left / (D / 2)
            # cmd_angular_right = cmd_linear_right / (D / 2)  

            # error_left = vel_left - cmd_angular_left
            # error_right = vel_right - cmd_angular_right
            # dt = (time_old - rospy.Time.now()).to_sec()
            # time_old = rospy.Time.now()
            # leftI += error_left * dt
            # rightI += error_right * dt
            # leftD = (error_left - leftErrorOld) / dt
            # rightD = (error_right - rightErrorOld) / dt
            # leftErrorOld = error_left
            # rightErrorOld = error_right

            # left_pid = Kp * error_left + Ki * leftI + Kd * leftD
            # right_pid = Kp * error_right + Ki * rightI + Kd * rightD
            cmd_angular_left, cmd_angular_right = wheel_vel.cmd_vel2angular_wheel_velocity(vel)
            dir_left, dir_right = wheel_vel.direction(cmd_angular_left, cmd_angular_right)
            if(dir_left):
                pid_left.pid_reset(True)
            if(dir_right):
                pid_right.pid_reset(True)
            left_pid = pid_left.PID_func(vel_left, cmd_angular_left, 0.0, 127)
            right_pid = pid_right.PID_func(vel_right, cmd_angular_right, 0.0, 127)

            msg_left = Int8()
            msg_right = Int8()

            msg_left.data = np.int8(left_pid)
            msg_right.data = np.int8(right_pid)
            pub_left_vel.publish(msg_left)
            pub_right_vel.publish(msg_right)
            print("msg_left: ", msg_left, " msg_right: ", msg_right)
        elif(b"Arduino exceptions" not in var[-1]):
            msg_left = Int8()
            msg_right = Int8()
            msg_left.data = np.int8(0)
            msg_right.data = np.int8(0)
            pub_left_vel.publish(msg_left)
            pub_right_vel.publish(msg_right)
            print("msg_left: ", msg_left, " msg_right: ", msg_right)


def reset_cov_cb(empty):
    send = str(str("true;") + str("null"))
    ser.write(bytes(send, encoding='utf8'))
    rospy.loginfo("dead reckoning covariance reset, done.")
    return EmptyResponse()


pub_left_vel = rospy.Publisher("/blattoidea/cmd_left", Int8, queue_size=1)
pub_right_vel = rospy.Publisher("/blattoidea/cmd_right", Int8, queue_size=1)
pid_left = PIDClass(50, 20, 0)
pid_right = PIDClass(50, 20, 0)
wheel_vel = wheelVelocity()

rospy.Subscriber("/cmd_vel", Twist, lambda msg: cmd_vel_cb(msg, ser, pid_left, pid_right, wheel_vel, pub_left_vel, pub_right_vel))
rospy.Service("/blattoidea/reset_dead_reckoning_cov", Empty, reset_cov_cb)

rospy.spin()
