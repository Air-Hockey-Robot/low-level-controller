import time
import math
from roboclaw_3 import Roboclaw
from AirHockeyTable import AirHockeyTable
import numpy as np

controller = AirHockeyTable("COM4", "COM5")

def get_target_pos(a, t):
    return a[0] + a[1]*t + a[2]*t^2 + a[3]*t^3


if __name__ == '__main__':
    controller.set_max_current(10)

    # controller.command_position(0.05, 0) # move 5 cm to the left
    # controller.log_current_state()

    #Stop motors
    controller.motor_l.ForwardM1(controller.addr_l,0)
    controller.motor_r.ForwardM1(controller.addr_r,0)

    #Set PID
    controller.set_left_motor_PID(200, 0, 5, 0, 0, -10000, 10000)
    controller.set_right_motor_PID(200, 0, 5, 0, 0, -10000, 10000)

    time.sleep(1)
    controller.zero_encoders()

    t0 = 0
    tf = 5

    mat = np.array([[1, t0, t0^2, t0^3],
            [0, 1, 2*t0, 3*t0^2],
            [1, tf, tf^2, tf^3],
            [0, 1, 2*tf, 3*tf^2]])

    start_position = controller.read_position()
    x0 = start_position[0]
    xf = 0.2
    vx0 = 0
    vxf = 0

    b = np.array([x0, vx0, xf, vxf])

    ax = np.linalg.solve(mat, b)

    y0 = start_position[1]
    yf = 0.2
    vy0 = 0
    vyf = 0

    b = np.array([y0, vy0, yf, vyf])

    ay = np.linalg.solve(mat, b)
    # time.sleep(1)

    # controller.home_table(position_threshold=100, x_speed=15, y_speed = 8)
    # controller.log_current_state()

    start_time = time.time()

    print(ax)
    print(ay)

    time.sleep(1000000)

    while(1):
        controller.log_current_state()
        current_time = time.time() - start_time

        x_pos = get_target_pos(ax, current_time)
        y_pos = get_target_pos(ay, current_time)

        controller.command_position(x_pos, y_pos)
        time.sleep(0.005)
        print(controller.read_motor_controller_voltages())