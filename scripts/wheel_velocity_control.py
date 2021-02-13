#!/usr/bin/env python3

class PIDClass:
    def __init__(self, Kp=50.0, Ki=0.9, Kd=0.0, bais=0.0, Ti=1.0):
        self.bais = bais
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.Ti = Ti
        self.ErrorOld = 0.0
        self.setpoint_tmp = 0.0

    def PID_func(self, measured_value, setpoint, dt, abs_max_pid_val=0.0, velocity=True, position=False, reset_integral=0.0):
        if(not velocity):
            self.measured_value = measured_value
            self.setpoint = setpoint
            if(self.setpoint_tmp != 0.0):
                error = self.setpoint_tmp - self.measured_value
                self.setpoint_tmp = 0.0
            else:
                error = self.setpoint - self.measured_value
            print("error: ", error)
            self.integral += error * dt
            derivative = (error - self.ErrorOld) / dt
            self.ErrorOld = error

            if(position):
                pid = self.bais + self.Kp * error + self.Ki * self.integral
                if(abs(pid) > abs_max_pid_val and abs_max_pid_val != 0.0 and position):
                    if(pid > 0):
                        pid = abs_max_pid_val
                    elif(pid < 0):
                        pid = -abs_max_pid_val
                    self.setpoint_tmp = abs_max_pid_val
                    error = self.setpoint_tmp - self.measured_value
                    self.integral = (1 / self.Ki) * (self.setpoint - self.bais - (self.Kp * error))
            else:
                pid = self.bais + self.Kp * error + self.Ki * self.integral + self.Kd * derivative
                if(abs(self.integral) >= reset_integral and reset_integral != 0.0):
                    self.integral = 0.0
                    pid = self.bais + self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        else:
            error = self.setpoint - self.measured_value
            pid = (self.Kp * ((1 + (self.dt / self.Ti)) * error)) - (self.Kp * self.ErrorOld)
            self.ErrorOld = error

        if(abs(pid) > abs_max_pid_val and abs_max_pid_val != 0.0):
            if(pid > 0):
                pid = abs_max_pid_val
            elif(pid < 0):
                pid = -abs_max_pid_val

        return pid

    def pid_reset(self, bool):
        if(bool):
            self.integral = 0.0
            self.ErrorOld = 0.0
            self.measured_value = self.setpoint
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
        print("cmd_angular_left: ", cmd_angular_left, "cmd_angular_right: ", cmd_angular_right)
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
