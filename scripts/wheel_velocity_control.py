#!/usr/bin/env python3

import rospy

class PIDClass:
    def __init__(self, Kp =  45.0, Ki = 13.5, Kd = 0.0):
        rospy.init_node("pid_node", anonymous=True)
        self.time_old = rospy.Time.now()
        self.Kp =  Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.ErrorOld = 0.0

    def PID_func(self, current, desirable, pid_val2reset_integral = 0.0, abs_max_pid_val = 0.0):
        error = current - desirable 
        dt = (self.time_old - rospy.Time.now()).to_sec()
        self.time_old = rospy.Time.now()
        self.integral += error * dt
        derivative = (error - self.ErrorOld) / dt
        self.ErrorOld = error

        pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        if (abs(pid) >= pid_val2reset_integral and pid_val2reset_integral is not 0.0):
            self.integral = 0.0
            pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        if (abs(pid) > abs_max_pid_val and abs_max_pid_val is not 0.0):
            if (pid > 0):
                pid = abs_max_pid_val
            elif (pid < 0):
                pid = -abs_max_pid_val

        return pid

def cmd_vel2angular_wheel_velocity(vel, diameter = 0.1651, wheelsSeparation = 0.42):
        cmd_linear_left = vel.linear.x + ((vel.angular.z * wheelsSeparation) / 2)
        cmd_linear_right = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2) 
        cmd_angular_left = cmd_linear_left / (diameter / 2)
        cmd_angular_right = cmd_linear_right / (diameter / 2)
        return cmd_angular_left, cmd_angular_right