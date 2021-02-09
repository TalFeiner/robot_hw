#!/usr/bin/env python3

import rospy


class PIDClass:
    def __init__(self, Kp=45.0, Ki=13.5, Kd=0.0):
        self.time_old = rospy.Time.now()
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.ErrorOld = 0.0

    def PID_func(self, current, desirable, pid_val2reset_integral=0.0, abs_max_pid_val=0.0):
        error = current - desirable
        dt = (self.time_old - rospy.Time.now()).to_sec()
        self.time_old = rospy.Time.now()
        self.integral += error * dt
        derivative = (error - self.ErrorOld) / dt
        self.ErrorOld = error

        pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        if (abs(pid) >= pid_val2reset_integral and pid_val2reset_integral != 0.0):
            self.integral = 0.0
            pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        if (abs(pid) > abs_max_pid_val and abs_max_pid_val != 0.0):
            if (pid > 0):
                pid = abs_max_pid_val
            elif (pid < 0):
                pid = -abs_max_pid_val

        return pid

    def pid_reset(self, bool):
        if(bool):
            self.integral = 0.0
            self.ErrorOld = 0.0
        else:
            pass


class wheelVelocity:
    def __init__(self):
        self.direction_left_old = True
        self.direction_right_old = True

    def cmd_vel2angular_wheel_velocity(self, vel, diameter=0.1651, wheelsSeparation=0.42):
        cmd_linear_left = vel.linear.x + ((vel.angular.z * wheelsSeparation) / 2.0)
        cmd_linear_right = vel.linear.x - ((vel.angular.z * wheelsSeparation) / 2.0)
        cmd_angular_left = cmd_linear_left / (diameter / 2)
        cmd_angular_right = cmd_linear_right / (diameter / 2)

        return cmd_angular_left, cmd_angular_right

    def direction(self, cmd_angular_left, cmd_angular_right):
        if(cmd_angular_left >= 0.0):
            direction_left = True
        elif(cmd_angular_left < 0.0):
            direction_left = False

        if(cmd_angular_right >= 0.0):
            direction_right = True
        elif(cmd_angular_right < 0.0):
            direction_right = False

        if(direction_left != self.direction_left_old):
            change_left = True
            self.direction_left_old = direction_left
        else:
            change_left = False

        if(direction_right != self.direction_right_old):
            change_right = True
            self.direction_right_old = direction_right
        else:
            change_right = False

        return change_left, change_right
