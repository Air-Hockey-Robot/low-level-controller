from roboclaw_3 import Roboclaw
from datetime import datetime
from math import pi

import time

class AirHockeyTable:
    BAUD_RATE = 460800
    PULLEY_RADIUS = 0.035 # in meters
    TICKS_PER_REV = 2048 # encoder ticks per motor revolution

    def __init__(self, com_port_l, com_port_r, addr_l=0x80, addr_r=0x85):
        self.motor_l = Roboclaw(com_port_l, self.BAUD_RATE)
        self.motor_r = Roboclaw(com_port_r, self.BAUD_RATE)
        self.addr_l = addr_l
        self.addr_r = addr_r

        self.target_x = 0
        self.target_y = 0

        log_file_name = datetime.now().strftime('log_%Y%m%d-%H%M%S.csv')
        self.log_handle = open(log_file_name, 'a')

        header_line = 'time (ns),actual theta1 (ticks),actual theta2 (ticks),target x (m),target y (m)'
        self.log_handle.write(header_line)
    
    def _theta2xy(self, theta_l, theta_r):
        '''Takes motor angles in encoder ticks (2048 ticks per revolution)
        and converts them into cartesian position of the mallet in meters'''
        x = (theta_l + theta_r) * self.PULLEY_RADIUS * pi / self.TICKS_PER_REV
        y = (theta_l - theta_r) * self.PULLEY_RADIUS * pi / self.TICKS_PER_REV
        
        return x, y

    def _xy2theta(self, x, y):
        '''Takes cartesian position of the mallet (x, y) in meters and converts
        it into motor angles in encoder ticks (2048 ticks per revolution)'''
        theta_l = (x + y) / self.PULLEY_RADIUS * self.TICKS_PER_REV / (2 * pi)
        theta_r = (x - y) / self.PULLEY_RADIUS * self.TICKS_PER_REV / (2 * pi)

        return theta_l, theta_r

    def command_position(self, x, y):
        '''Command motors to move the mallet to the desired cartesian position.
        Position is interpreted in meters'''
        theta_l, theta_r = self._xy2theta(x, y)

        self.motor_l.SpeedAccelDeccelPositionM1(self.addr_l, 0, 0, 0, theta_l)
        self.motor_r.SpeedAccelDeccelPositionM1(self.addr_r, 0, 0, 0, theta_r)

        self.target_x = x
        self.target_y = y
    
    def log_current_state(self):
        '''Write a line to the log file containing information about the state of the system'''
        # might need to use the other elements of the returned tuple
        # see: https://python-roboclaw.readthedocs.io/en/latest/api.html#roboclaw.roboclaw.Roboclaw.read_encoder_m1
        enc_l = self.motor_l.ReadEncM1(self.addr_l)
        enc_r = self.motor_r.ReadEncM1(self.addr_r)
        
        line = f'{time.time_ns()},{enc_l[1]},{enc_r[1]},{self.target_x},{self.target_y}'
        self.log_handle.write(line)

    def zero_encoders(self):
        '''Set the current encoder positions as the zero reference'''
        self.motor_l.SetEncM1(self.addr_l, 0)
        self.motor_r.SetEncM1(self.addr_r, 0)

    def set_left_motor_PID(self, kp, ki, kd, kimax, deadzone, min, max):
        '''Sets PID gains for position controller of left motor'''
        self.motor_l.SetM1PositionPID(self.addr_l, kp, ki, kd, kimax, deadzone, min, max)
    
    def set_right_motor_PID(self, kp, ki, kd, kimax, deadzone, min, max):
        '''Sets PID gains for position controller of right motor'''
        self.motor_r.SetM1PositionPID(self.addr_l, kp, ki, kd, kimax, deadzone, min, max)

    def read_left_motor_PID(self):
        '''Reads the PID parameters for the position controller of the left motor.
        Output order: (kp, ki, kd, kimax, deadzone, min, max)'''
        return self.motor_l.ReadM1PositionPID(self.addr_l)
        
    def read_right_motor_PID(self):
        '''Reads the PID parameters for the position controller of the right motor
        Output order: (kp, ki, kd, kimax, deadzone, min, max)'''
        return self.motor_r.ReadM1PositionPID(self.addr_r)


