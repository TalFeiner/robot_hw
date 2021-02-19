#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_srvs.srv import Empty, EmptyResponse
import serial
from serial.tools import list_ports
import numpy as np
from wheel_velocity_control import PIDClass, wheelVelocity

pid_duration = 0.1
encoder_duration = 0.000001
bais = 0.0
Kp = 10.0
Ki = 1.0
Kd = 0.0
Ti = 1.0


class blattoideaBridge(wheelVelocity):

    def __init__(self, Kp=50.0, Ki=0.9, Kd=0.0, bais=0.0, Ti=1.0):
        super().__init__()

        rospy.init_node("blattoidea_hw_node", anonymous=True)
        self.pid_left = PIDClass(Kp, Ki, Kd, bais, Ti)
        self.pid_right = PIDClass(Kp, Ki, Kd, bais, Ti)
        self.time_old = rospy.Time.now()
        self.pub_left_vel = rospy.Publisher("/blattoidea/cmd_left", Int8, queue_size=1)
        self.pub_right_vel = rospy.Publisher("/blattoidea/cmd_right", Int8, queue_size=1)
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_angular_left, self.cmd_angular_right = self.cmd_vel2angular_wheel_velocity(vel)
        self.left_pid = self.pid_left.PID(0.0, self.cmd_angular_left, pid_duration, 127)
        self.right_pid = self.pid_right.PID(0.0, self.cmd_angular_right, pid_duration, 127)

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
                self.ser = serial.Serial(port[0], baud)  # open serial port
                # print(ser.name)         # check which port was really used
            else:
                rospy.logerr("Error port is not found")

        else:
            self.ser = serial.Serial(port, baud)

        rospy.Timer(rospy.Duration(pid_duration), self.encoder_timer_cb)
        rospy.wait_for_message("/cmd_vel", Twist)
        rospy.Timer(rospy.Duration(pid_duration), self.pid_timer_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Service("/blattoidea/reset_dead_reckoning_cov", Empty, self.reset_cov_cb)

    def encoder_timer_cb(self, event):
        line = self.ser.readline()
        # print("line: ", line)
        self.var = line.split(b";")

    def cmd_vel_cb(self, vel):
        self.cmd_angular_left, self.cmd_angular_right = self.cmd_vel2angular_wheel_velocity(vel)
        dir_left, dir_right = self.direction(self.cmd_angular_left, self.cmd_angular_right)
        if(dir_left):
            self.pid_left.pid_reset(True)
        if(dir_right):
            self.pid_right.pid_reset(True)

        if(vel.linear.x == 0.0 and vel.angular.z == 0.0):
            msg_left = Int8()
            msg_right = Int8()
            msg_left.data = np.int8(0)
            msg_right.data = np.int8(0)
            self.pub_left_vel.publish(msg_left)
            self.pub_right_vel.publish(msg_right)
            self.pid_left.pid_reset(True)
            self.pid_right.pid_reset(True)
        elif(self.left_pid is None or self.right_pid is None):
            msg_left = Int8()
            msg_right = Int8()
            msg_left.data = np.int8(0)
            msg_right.data = np.int8(0)
            self.pub_left_vel.publish(msg_left)
            self.pub_right_vel.publish(msg_right)
            self.pid_left.pid_reset(True)
            self.pid_right.pid_reset(True)
        else:
            msg_left = Int8()
            msg_right = Int8()
            msg_left.data = np.int8(self.left_pid)
            msg_right.data = np.int8(self.right_pid)
            self.pub_left_vel.publish(msg_left)
            self.pub_right_vel.publish(msg_right)
            print("msg_left: ", msg_left, " msg_right: ", msg_right)

    def pid_timer_cb(self, event):
        if (len(self.var) > 2 and b"Arduino exceptions" not in self.var[-1]):
            vel_left = float(self.var[1])
            vel_right = -float(self.var[2])
            print("vel_left: ", vel_left, " vel_right: ", vel_right)
            dt = (event.current_real - self.time_old).to_sec()
            self.time_old = event.current_real
            self.left_pid = self.pid_left.PID(vel_left, self.cmd_angular_left, dt, 127, 100)
            self.right_pid = self.pid_right.PID(vel_right, self.cmd_angular_right, dt, 127, 100)

        elif(b"Arduino exceptions" not in self.var[-1]):
            msg_left = Int8()
            msg_right = Int8()
            msg_left.data = np.int8(0)
            msg_right.data = np.int8(0)
            self.pub_left_vel.publish(msg_left)
            self.pub_right_vel.publish(msg_right)
            rospy.logerr("Arduino Error: " + "msg_left: ", msg_left, " msg_right: ", msg_right)

    def reset_cov_cb(self, empty):
        send = str(str("true;") + str("null"))
        self.ser.write(bytes(send, encoding='utf8'))
        rospy.loginfo("dead reckoning covariance reset, done.")
        return EmptyResponse()


blattoideaBridge(Kp, Ki, Kd, bais, Ti)
rospy.spin()
