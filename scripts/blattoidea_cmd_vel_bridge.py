#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import serial
from serial.tools import list_ports
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

global ser, ser2, debug, odom_pub, seq, count_cmd_cb, map_tf_flag
debug = False
map_tf_flag = False
seq = 0


def odom(line):
    global odom_pub, seq
    if "Odom" in line:
        line_list = line.split(";")
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
        odom_msg.child_frame_id = "world"
        odom_msg.header.seq = seq
        odom_msg.header.stamp = rospy.Time.now()
        if map_tf_flag:
            odom_msg.header.frame_id = "map"
        else:
            odom_msg.header.frame_id = "world"
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
        odom_pub.publish(odom_msg)
        seq += 1


def cmd_vel2angular_wheel_velocity(vel, diameter=0.1651, wheelsSeparation=0.42):
    cmd_linear_left = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2.0)
    cmd_linear_right = vel.linear.x + ((vel.angular.z * wheelsSeparation) / 2.0)
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


def map_tf_flag_cb(flag):
    global map_tf_flag
    map_tf_flag = flag.data


def cmd_vel_cb(vel):
    global ser, count_cmd_cb
    cmd_angular_left, cmd_angular_right = cmd_vel2angular_wheel_velocity(vel)
    cmd_angular_left = (cmd_angular_left)
    cmd_angular_right = (cmd_angular_right)
    send = str(str("cmdVel") + str(";") + str(cmd_angular_left) + str(";") + str(cmd_angular_right) + '\n')
    try:
        ser.write(bytes(send, encoding='utf8'))
    except serial.SerialException as e:
        rospy.logerr(e)
        pass
    if(debug):
        print("sending: " + str(send))
    count_cmd_cb += 1
    if(count % 10 == 0):
        ser.flushOutput()
        count_cmd_cb = 0


def reset_cov_cb(empty):
    global ser
    send = str("resetError;null")
    for __ in range(3):
        ser.write(bytes(send, encoding='utf8'))
        rospy.sleep(0.1)
    rospy.loginfo("dead reckoning covariance reset, done.")
    return EmptyResponse()


rospy.init_node("blattoidea_hw_node", anonymous=True)
rospy.Service("/reset_dead_reckoning_cov", Empty, reset_cov_cb)
rospy.Subscriber("/map_tf_frame_exist_flag", Bool, map_tf_flag_cb)
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=4)
count_cmd_cb = 0
count = 0
open_serial_port()
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)
rospy.loginfo("Blattoidea is under your command")
while not rospy.is_shutdown():
    if(ser2.in_waiting > 0):
        try:
            line = ser2.readline().decode('utf-8')
            odom(line)
            if(debug):
                print("msg - ", line)
        except serial.SerialException as e:
            rospy.logerr(e)
            pass
    rospy.sleep(0.005)
    count += 1
    if(count % 10 == 0):
        ser2.flushInput()
        count = 0
