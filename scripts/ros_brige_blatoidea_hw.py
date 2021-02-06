#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from  std_srvs.srv import Empty, EmptyResponse
import serial
from serial.tools import list_ports
import numpy as np
# import string

global time_old, Kp, Ki, Kd, linearI, angularI, linearErrorOld, angularErrorOld
Kp = 85.0; Ki = 60; Kd = 25
linearI = 0.0; angularI = 0.0
linearErrorOld = 0.0; angularErrorOld = 0.0

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

pub_linear_vel = rospy.Publisher("/blattoidea/cmd_pwm_linear", Int8, queue_size = 1)
pub_angular_vel = rospy.Publisher("/blattoidea/cmd_pwm_angular", Int8, queue_size = 1)
time_old = rospy.Time.now()

def cmd_vel_cb (vel):
    global time_old, Kp, Ki, Kd, linearI, angularI, linearErrorOld, angularErrorOld

    line = ser.readline()
    # try:
    #     line = ''.join(filter(lambda c: c in string.printable, line))
    # except:
    #     pass
    # print("line: ", line)
    var = line.split(b";")
    if (len(var) > 2 and not b"Arduino exceptions" in var[-1]):
        linearVel = float(var[1])
        angularVel = float(var[2])
        print ("linearVel: ", linearVel, " angularVel: ", angularVel)

        linearError = vel.linear.x - linearVel
        angularError = vel.angular.z - angularVel
        dt = (time_old - rospy.Time.now()).to_sec()
        time_old = rospy.Time.now()
        linearI += linearError * dt
        angularI += angularError * dt
        linearD = (linearError - linearErrorOld) / dt
        angularD = (angularError - angularErrorOld) / dt
        linearErrorOld = linearError
        angularErrorOld = angularError

        pwm_linear = Int8()
        pwm_angular = Int8()
        pwm_l = np.int8(Kp * linearError + Ki * linearI + Kd * linearD)
        pwm_a = np.int8(Kp * angularError + Ki * angularI + Kd * angularD)
        
        if (pwm_l > 127):
            pwm_l = 127
        if (pwm_l < -127):
            pwm_l = -127

        if (pwm_a > 127):
            pwm_a = 127
        if (pwm_a < -127):
            pwm_a = -127

        pwm_linear.data = np.int8(pwm_l)
        pwm_angular.data = np.int8(pwm_a)
        pub_linear_vel.publish(pwm_linear)
        pub_angular_vel.publish(pwm_angular)
    elif (not b"Arduino exceptions" in var[-1]):
        pwm_linear = Int8()
        pwm_angular = Int8()
        pwm_linear.data = np.int8(0)
        pwm_angular.data = np.int8(0)
        pub_linear_vel.publish(pwm_linear)
        pub_angular_vel.publish(pwm_angular)

def reset_cov_cb (empty):
    send = str(str("true;") + str("null"))
    ser.write(bytes(send, encoding='utf8'))
    rospy.loginfo("dead reckoning covariance reset, done.")
    return EmptyResponse()

rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)
rospy.Service("/blattoidea/reset_dead_reckoning_cov", Empty, reset_cov_cb)

rospy.spin()
