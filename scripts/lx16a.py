#!/usr/bin/env python3

import serial
import lewansoul_lx16a
import rospy
import numpy as np
from blattoidea_hw.srv import semanticSlam, semanticSlamResponse

SERIAL_PORT = '/dev/ttyUSB0'
servo_id = [1, 2]
angles_names = ['pitch', 'yaw']
epsilon = 0.0085  # radian


def pulse_per_rev2radian(pulse):
    total_pulses = 1000  # for 2/3 radian
    max_radian = 2/3 * np.pi  # 2/3 radian
    return (max_radian / total_pulses) * pulse


def radian2pulse_per_rev(radian):
    total_pulses = 1000  # for 2/3 radian
    max_radian = 2/3 * np.pi  # 2/3 radian
    return int((radian / max_radian) * total_pulses)


def check(req_tmp):
    total_pulses = 1000  # for 2/3 radian
    max = pulse_per_rev2radian(total_pulses)
    if radian2pulse_per_rev(req_tmp) > 1000:
        req_tmp = max
        rospy.logerr("Request limits are (0, %s)[rad]. you requested: %s, this is beyond limit. changing your request to %s [rad].", max, req_tmp, max)
    elif radian2pulse_per_rev(req_tmp) < 0:
        req_tmp = 0
        rospy.logerr("Request limits are (0, %s)[rad]. you requested: %s, this is beyond limit. changing your request to 0 [rad].", max, req_tmp)
    return req_tmp


def semantic_slam_cb(req, controller, servo, angle):
    for ii in range(len(servo)):
        setattr(req, angle[ii], check(getattr(req, angle[ii])))
        servo[ii].move_prepare(radian2pulse_per_rev(getattr(req, angle[ii])))
    controller.move_start()

    res = semanticSlamResponse()
    res.success = False
    while not res.success:
        for ii in range(len(servo)):
            setattr(res, angle[ii], pulse_per_rev2radian(
                    servo[ii].get_position(getattr(req, angle[ii])))
                    )
            print("dist: ", abs(getattr(res, angle[ii]) - getattr(req, angle[ii])))
            if abs(getattr(res, angle[ii]) - getattr(req, angle[ii])) > epsilon:
                res.success = False
                break
            else:
                res.success = True
        if not res.success:
            rospy.sleep(0.5)
            controller.move_start()
            continue
        rospy.sleep(0.1)
        for ii in range(len(servo)):
            setattr(res, angle[ii], pulse_per_rev2radian(
                    servo[ii].get_position(getattr(req, angle[ii])))
                    )
    return res


rospy.init_node('semantic_slam_sensor_node')
controller = lewansoul_lx16a.ServoController(
    serial.Serial(SERIAL_PORT, 115200, timeout=1),
    )
servo_list = []
for ii in range(len(servo_id)):
    servo_list.append(controller.servo(servo_id[ii]))
    if controller.is_motor_on(servo_id[ii]):
        rospy.loginfo("Servo motor, ID %s is on", servo_id[ii])
    else:
        rospy.logerr("Servo motor, ID %s is OFF!", servo_id[ii])
args = [controller, servo_list, angles_names]
rospy.Service('semantic_slam', semanticSlam, lambda req: semantic_slam_cb(req, *args))
rospy.loginfo("Semantic SLAM sensor is ready to scan")
rospy.spin()
