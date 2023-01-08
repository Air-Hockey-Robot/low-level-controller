from roboclaw_3 import Roboclaw
from datetime import datetime
from math import pi
from numpy import NaN

import time

class AirHockeyTable:
    BAUD_RATE = 460800
    PULLEY_RADIUS = 0.035 # in meters
    TICKS_PER_REV = 2048 # encoder ticks per motor revolution

    def __init__(self, com_port_l, com_port_r, addr_l=0x80, addr_r=0x85):
        self.start_time = time.time()

        self.motor_l = Roboclaw(com_port_l, self.BAUD_RATE)
        self.motor_r = Roboclaw(com_port_r, self.BAUD_RATE)

        self.motor_l.Open()
        self.motor_r.Open()

        self.addr_l = addr_l
        self.addr_r = addr_r

        self.target_x = NaN
        self.target_y = NaN

        log_file_name = datetime.now().strftime('log_%Y%m%d-%H%M%S.csv')
        self.log_handle = open('data/' + log_file_name, 'a')

        header_line = 'time (s),actual x (cm),actual y (cm),target x (cm),target y (cm),' \
            'left motor current (A),right motor current (A),'\
            'left motor controller voltage (V),right motor controller voltage (V)\n'
        self.log_handle.write(header_line)
    
    def _theta_to_xy(self, theta_l, theta_r):
        '''Takes motor angles in encoder ticks (2048 ticks per revolution)
        and converts them into cartesian position of the mallet in meters'''
        x = (theta_l + theta_r) * self.PULLEY_RADIUS * pi / self.TICKS_PER_REV
        y = (theta_l - theta_r) * self.PULLEY_RADIUS * pi / self.TICKS_PER_REV
        
        return -x, -y

    def _xy_to_theta(self, x, y):
        '''Takes cartesian position of the mallet (x, y) in meters and converts
        it into motor angles in encoder ticks (2048 ticks per revolution)'''
        theta_l = (x + y) / self.PULLEY_RADIUS * self.TICKS_PER_REV / (2*pi)
        theta_r = (x - y) / self.PULLEY_RADIUS * self.TICKS_PER_REV / (2*pi)

        return -theta_l, -theta_r

    def command_position(self, x, y):
        '''Command motors to move the mallet to the desired cartesian position.
        Position is interpreted in meters'''
        theta_l, theta_r = self._xy_to_theta(x, y)

        self.motor_l.SpeedAccelDeccelPositionM1(self.addr_l, 400000, 350000, 400000, int(round(theta_l)), 0)
        self.motor_r.SpeedAccelDeccelPositionM1(self.addr_r, 400000, 350000, 400000, int(round(theta_r)), 0)

        self.target_x = x
        self.target_y = y
    
    def log_current_state(self):
        '''Write a line to the log file containing information about the state of the system'''
        # might need to use the other elements of the returned tuple
        # see: https://python-roboclaw.readthedocs.io/en/latest/api.html#roboclaw.roboclaw.Roboclaw.read_encoder_m1
        enc_l = self.motor_l.ReadEncM1(self.addr_l)[1]
        enc_r = self.motor_r.ReadEncM1(self.addr_r)[1]
        
        actual_x, actual_y = self._theta_to_xy(enc_l, enc_r)

        current_l, current_r = self.read_motor_currents()

        voltage_l, voltage_r = self.read_motor_controller_voltages()

        line = f'{time.time() - self.start_time},{actual_x*100},{actual_y*100},{self.target_x*100},' \
            f'{self.target_y*100},{current_l},{current_r},{voltage_l},{voltage_r}\n'
        self.log_handle.write(line)

    def zero_encoders(self):
        '''Set the current encoder positions as the zero reference'''
        self.motor_l.SetEncM1(self.addr_l, 0)
        self.motor_r.SetEncM1(self.addr_r, 0)

    def read_encoders(self):
        return self.motor_l.ReadEncM1(self.addr_l)[1], self.motor_r.ReadEncM1(self.addr_r)[1]
    
    def read_position(self):
        encoder_values = self.read_encoders()
        return self._theta_to_xy(encoder_values[0], encoder_values[1])

    def set_left_motor_PID(self, kp, ki, kd, kimax, deadzone, min, max):
        '''Sets PID gains for position controller of left motor'''
        self.motor_l.SetM1PositionPID(self.addr_l, kp, ki, kd, kimax, deadzone, min, max)
    
    def set_right_motor_PID(self, kp, ki, kd, kimax, deadzone, min, max):
        '''Sets PID gains for position controller of right motor'''
        self.motor_r.SetM1PositionPID(self.addr_r, kp, ki, kd, kimax, deadzone, min, max)

    def read_left_motor_PID(self):
        '''Reads the PID parameters for the position controller of the left motor.
        Output order: (kp, ki, kd, kimax, deadzone, min, max)'''
        return self.motor_l.ReadM1PositionPID(self.addr_l)
        
    def read_right_motor_PID(self):
        '''Reads the PID parameters for the position controller of the right motor
        Output order: (kp, ki, kd, kimax, deadzone, min, max)'''
        return self.motor_r.ReadM1PositionPID(self.addr_r)

    def read_motor_currents(self):
        '''Reads motor currents'''
        current_l = self.motor_l.ReadCurrents(self.addr_l)[1] / 100
        current_r = self.motor_r.ReadCurrents(self.addr_r)[1] / 100
        return current_l, current_r

    def read_motor_controller_voltages(self):
        '''Reads motor controller input voltage'''
        voltage_l = self.motor_l.ReadMainBatteryVoltage(self.addr_l)[1] / 10
        voltage_r = self.motor_r.ReadMainBatteryVoltage(self.addr_r)[1] / 10

        return voltage_l, voltage_r

    def set_max_current(self, max_current):
        '''Set max motor current'''
        self.motor_l.SetM1MaxCurrent(self.addr_l, max_current*100)
        self.motor_r.SetM1MaxCurrent(self.addr_r, max_current*100)

    def home_table(self, x_speed, y_speed, position_threshold):
        '''Home the table and zero the econders'''
        threshold_count = 0

        self.motor_l.ForwardM1(self.addr_l, x_speed)
        self.motor_r.ForwardM1(self.addr_r, x_speed)
        self.log_current_state()
        time.sleep(0.5)

        while True:
            self.log_current_state()
            last_encoder_pos = self.motor_l.ReadEncM1(self.addr_l)[1]
            time.sleep(0.1)

            if abs(self.motor_l.ReadEncM1(self.addr_l)[1] - last_encoder_pos) < position_threshold:
                self.motor_l.ForwardM1(self.addr_l, 0)
                self.motor_r.ForwardM1(self.addr_r, 0)
                print("Homed in X")
                break
        
            
        self.motor_l.ForwardM1(self.addr_l, y_speed)
        self.motor_r.BackwardM1(self.addr_r, y_speed)
        self.log_current_state()
        time.sleep(0.5)

        while True:
            self.log_current_state()
            last_encoder_pos = self.motor_l.ReadEncM1(self.addr_l)[1]
            time.sleep(0.1)

            if abs(self.motor_l.ReadEncM1(self.addr_l)[1] - last_encoder_pos) < position_threshold:
                self.motor_l.ForwardM1(self.addr_l, 0)
                self.motor_r.ForwardM1(self.addr_r, 0)
                print("Homed in Y")
                break

        self.motor_l.ForwardM1(self.addr_l, x_speed)
        time.sleep(0.5)
        self.motor_l.ForwardM1(self.addr_l, 0)
        self.zero_encoders()
        print("Fully Homed")


    def check_path_bounds(self, x_coordinates, y_coordinates, x_min, x_max, y_min, y_max):
        '''Check whether a path is within the table's bounds'''

        return((all(x < x_max for x in x_coordinates) and all(x > x_min for x in x_coordinates)) and \
            (all(y < y_max for y in y_coordinates) and all(y > y_min for y in y_coordinates)))