import time
import math
from roboclaw_3 import Roboclaw
from AirHockeyTable import AirHockeyTable
import numpy as np

controller = AirHockeyTable("COM4", "COM5")

def get_target_pos(a, t):
    return sum([c * t**i for i, c in enumerate(a)])
    # return a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3

def get_intermediate_point(x, y, vx, vy, final_time, straight_length):
    final_velocity_magnitude = np.sqrt(vx**2 + vy**2)
    reverse_x_component = -vx / final_velocity_magnitude * straight_length
    reverse_y_component = -vy / final_velocity_magnitude * straight_length
    
    intermediate_x = x + reverse_x_component
    intermediate_y = y + reverse_y_component

    intermediate_t = final_time - straight_length / final_velocity_magnitude

    return intermediate_x, intermediate_y, intermediate_t

def get_trajectory_coeffs(t0,x0,y0,vx0,vy0,tf,xf,yf,vxf,vyf,d):
    x1, y1, t1 = get_intermediate_point(xf, yf, vxf, vyf, tf, 0.1)

    mat = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [1, t1, t1**2, t1**3, t1**4, t1**5],
        [0, 1, 2*t1, 3*t1**2, 4*t1**3, 5*t1**4],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4]])

    bx = np.array([x0, vx0, x1, vxf, xf, vxf])
    ax = np.linalg.solve(mat, bx)

    by = np.array([y0, vy0, y1, vyf, yf, vyf])
    ay = np.linalg.solve(mat, by)

    return ax, ay


if __name__ == '__main__':
    controller.set_max_current(25)
    controller.home_table(position_threshold=100, x_speed=15, y_speed = 8)

    #Stop motors
    controller.motor_l.ForwardM1(controller.addr_l,0)
    controller.motor_r.ForwardM1(controller.addr_r,0)

    #Set PID
    controller.set_left_motor_PID(100, 0, 0, 0, 0, -10000, 20000)
    controller.set_right_motor_PID(100, 0, 0, 0, 0, -10000, 20000)

    time.sleep(1)
    controller.zero_encoders()

    # First path

    t0 = 0
    start_position = controller.read_position()
    x0 = start_position[0]
    y0 = start_position[1]
    vx0 = 0
    vy0 = 0

    tf = 0.75
    xf = 0.5
    yf = 0.5
    vxf = 0
    vyf = 0.5

    ax, ay = get_trajectory_coeffs(t0,x0,y0,vx0,vy0,tf,xf,yf,vxf,vyf,0.1)

    x_traj = [get_target_pos(ax, t) for t in np.arange(0,tf,0.001)]
    y_traj = [get_target_pos(ay, t) for t in np.arange(0,tf,0.001)]

    if not controller.check_path_bounds(x_traj, y_traj, 0, 0.85, 0, 0.9):
        print("PATH VIOLATES TABLE BOUNDS")
        exit(0)

    start_time = time.time()

    while(time.time() < start_time + tf):
        controller.log_current_state()

        current_time = time.time() - start_time
        x_pos = get_target_pos(ax, current_time)
        y_pos = get_target_pos(ay, current_time)

        controller.command_position(x_pos, y_pos)

    # Second path

    t0 = 0.75
    start_position = controller.read_position()
    x0 = start_position[0]
    y0 = start_position[1]
    vx0 = 0
    vy0 = 0

    tf = 1.5
    xf = 0.5
    yf = 0.05
    vxf = 0
    vyf = -0.1

    ax, ay = get_trajectory_coeffs(t0,x0,y0,vx0,vy0,tf,xf,yf,vxf,vyf,0.1)

    x_traj = [get_target_pos(ax, t) for t in np.arange(0,tf,0.001)]
    y_traj = [get_target_pos(ay, t) for t in np.arange(0,tf,0.001)]

    from matplotlib import pyplot as plt
    plt.plot(x_traj, y_traj)
    plt.show()

    if not controller.check_path_bounds(x_traj, y_traj, 0, 0.85, 0, 0.9):
        print("PATH VIOLATES TABLE BOUNDS")
        exit(0)


    while(time.time() < start_time + tf):
        controller.log_current_state()

        current_time = time.time() - start_time
        x_pos = get_target_pos(ax, current_time)
        y_pos = get_target_pos(ay, current_time)

        controller.command_position(x_pos, y_pos)

    #Stop motors
    controller.motor_l.ForwardM1(controller.addr_l,0)
    controller.motor_r.ForwardM1(controller.addr_r,0)