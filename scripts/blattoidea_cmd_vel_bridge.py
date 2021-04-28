#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import serial
from serial.tools import list_ports
from std_srvs.srv import Empty, EmptyResponse
import threading
import string

global seq, count_cmd_cb, count
debug = False
seq = 0
count_cmd_cb = 0
count = 0


def odom_func(line, odom_pub):
    global seq
    if "Odom" in line:
        line_list = line.split(";")
        if (len(line_list) == 22 and line_list[-1] == "\n" and "angular" in
           line and "linear" in line and "x" in line and "y" in line and
           "theta" in line and "xVar" in line and "yVar" in line and
           "thetaVar" in line and "angularVar" in line and "linearVar" in line):
            try:
                angular_vel_idx = line_list.index("angular") + 1
                linear_vel_idx = line_list.index("linear") + 1
                x_pose_idx = line_list.index("x") + 1
                y_pose_idx = line_list.index("y") + 1
                theta_idx = line_list.index("theta") + 1
                angular_vel = line_list[angular_vel_idx]
                linear_vel = line_list[linear_vel_idx]
                x_pose = line_list[x_pose_idx]
                y_pose = line_list[y_pose_idx]
                theta = line_list[theta_idx]

                xVar_pose_idx = line_list.index("xVar") + 1
                yVar_pose_idx = line_list.index("yVar") + 1
                thetaVar_idx = line_list.index("thetaVar") + 1
                angularVar_vel_idx = line_list.index("angularVar") + 1
                linearVar_vel_idx = line_list.index("linearVar") + 1
                angularVar_vel = line_list[angularVar_vel_idx]
                linearVar_vel = line_list[linearVar_vel_idx]
                xVar_pose = line_list[xVar_pose_idx]
                yVar_pose = line_list[yVar_pose_idx]
                thetaVar = line_list[thetaVar_idx]

                quat = Rotation.from_euler('z', float(theta)).as_quat()

                # The pose in this message should be specified in the coordinate frame given by header.frame_id.
                # The twist in this message should be specified in the coordinate frame given by the child_frame_id
                odom_msg = Odometry()
                odom_msg.child_frame_id = "odom"
                odom_msg.header.seq = seq
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "odom"
                odom_msg.pose.pose.position.x = float(x_pose)
                odom_msg.pose.pose.position.x = float(y_pose)
                odom_msg.pose.pose.orientation = Quaternion(*quat)
                odom_msg.twist.twist.linear.x = float(linear_vel)
                odom_msg.twist.twist.angular.z = float(angular_vel)

                odom_msg.pose.covariance[0] = float(xVar_pose)
                odom_msg.pose.covariance[7] = float(yVar_pose)
                odom_msg.pose.covariance[35] = float(thetaVar)
                odom_msg.twist.covariance[0] = float(linearVar_vel)
                odom_msg.twist.covariance[35] = float(angularVar_vel)

            finally:
                odom_pub.publish(odom_msg)
                seq += 1


def cmd_vel2angular_wheel_velocity(vel, diameter=0.1651,
                                   wheelsSeparation=0.385):
    cmd_linear_left = vel.linear.x - ((vel.angular.z *
                                       wheelsSeparation) / 2.0)
    cmd_linear_right = vel.linear.x + ((vel.angular.z *
                                        wheelsSeparation) / 2.0)
    cmd_angular_left = cmd_linear_left / (diameter / 2)
    cmd_angular_right = cmd_linear_right / (diameter / 2)
    # print("cmd_angular_left: ", cmd_angular_left, "cmd_angular_right: ", cmd_angular_right)
    return cmd_angular_left, cmd_angular_right


def open_serial_port():
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

        if len(ports) == 2 and ("/dev/ttyUSB" in
                                port[0]) and ("/dev/ttyUSB" in port[1]):
            ser = serial.Serial(port[0], baud,
                                write_timeout=0.5, timeout=0.5)  # open serial port
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

        if len(ports) == 2 and ("/dev/ttyUSB" in
                                port[0]) and ("/dev/ttyUSB" in port[1]):
            ser2 = serial.Serial(port[1], baud,
                                 write_timeout=0.5, timeout=0.5)  # open serial port
            # print(ser.name)         # check which port was really used
        else:
            rospy.logerr("Error port is not found")

    else:
        ser2 = serial.Serial(port2, baud2, write_timeout=0.5, timeout=0.5)  # open serial port

    ser.flushInput()
    ser2.flushInput()
    ser.flushOutput()
    ser2.flushOutput()
    return ser, ser2


def cmd_vel_cb(vel, ser, debug, lock):
    global count_cmd_cb
    try:
        lock.acquire()
        cmd_angular_left, cmd_angular_right = cmd_vel2angular_wheel_velocity(vel)
        cmd_angular_left = (cmd_angular_left)
        cmd_angular_right = (cmd_angular_right)
        send = (str(str("cmdVel") + str(";") + str(cmd_angular_left) +
                str(";") + str(cmd_angular_right) + '\n'))
        try:
            ser.write(bytes(send, encoding='utf8'))
            if(debug):
                print("sending: " + str(send))
            count_cmd_cb += 1
            if(count_cmd_cb % 10 == 0):
                ser.flushOutput()
                count_cmd_cb = 0
        except serial.SerialException as e:
            rospy.logerr(e)
            pass
    finally:
        lock.release()
        return EmptyResponse()


def reset_cov_cb(empty, ser, lock):
    try:
        lock.acquire()
        send = str("resetError;null" + '\n')
        for __ in range(3):
            ser.write(bytes(send, encoding='utf8'))
            rospy.sleep(0.01)
        rospy.loginfo("dead reckoning covariance reset, done.")
    finally:
        lock.release()
        return EmptyResponse()


def emergency_stope_cb(empty, emergency_cmd_stope, ser, lock):
    try:
        lock.acquire()
        send = str("emergencyStope;null" + '\n')
        stope_msg = Twist()
        stope_msg.linear.x = 0
        stope_msg.linear.y = 0
        stope_msg.linear.z = 0
        stope_msg.angular.x = 0
        stope_msg.angular.y = 0
        stope_msg.angular.z = 0
        for __ in range(3):
            ser.write(bytes(send, encoding='utf8'))
            emergency_cmd_stope.publish(stope_msg)
            rospy.sleep(0.01)
        rospy.loginfo("Emergency stope!!!")
    finally:
        lock.release()
        return EmptyResponse()


def myhook(ser2, ser, lock):
    try:
        ser2.close
    finally:
        pass
    try:
        ser.close
    finally:
        pass
    try:
        lock.release()
    finally:
        pass
    rospy.loginfo("Bye, see you soon :)")


def main_cb(event, ser2, odom_pub, debug, lock):
    global count
    try:
        lock.acquire()
        try:
            if(ser2.in_waiting > 0):
                line = bytes(ser2.readline()).decode('utf-8')
                line = ''.join(filter(lambda c: c in string.printable, line))
                odom_func(line, odom_pub)
                if(debug):
                    print("msg - ", line)
            count += 1
            if(count % 10 == 0):
                ser2.flushInput()
                count = 0
        except serial.SerialException as e:
            rospy.logerr(e)
            pass
    finally:
        lock.release()


rospy.init_node("blattoidea_hw_node", anonymous=True)
lock = threading.Lock()
ser, ser2 = open_serial_port()
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=4)
rospy.Service("/reset_dead_reckoning_cov", Empty,
              lambda req: reset_cov_cb(req, ser, lock))
emergency_cmd_stope = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.Service("/emergency_stope", Empty,
              lambda rqs: emergency_stope_cb(rqs, emergency_cmd_stope,
                                             ser, lock))
rospy.Subscriber("/cmd_vel", Twist,
                 lambda msg: cmd_vel_cb(msg, ser, debug, lock))
rospy.loginfo("Blattoidea is under your command.")

rospy.Timer(rospy.Duration(0.1),
            lambda event: main_cb(event, ser2, odom_pub, debug, lock))
rospy.on_shutdown(lambda: myhook(ser2, ser, lock))
rospy.spin()
