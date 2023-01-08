import time
import math
from roboclaw_3 import Roboclaw
from AirHockeyTable import AirHockeyTable

controller = AirHockeyTable("COM4", "COM5")

if __name__ == '__main__':
    controller.set_max_current(10)

    # controller.command_position(0.05, 0) # move 5 cm to the left
    # controller.log_current_state()

    #Inputs
    period = 3 #s
    angular_frequency = 2 * math.pi / period #rad/s
    travel_distance = 0.10 #m

    #Stop motors
    controller.motor_l.ForwardM1(controller.addr_l,0)
    controller.motor_r.ForwardM1(controller.addr_r,0)

    #Set PID
    controller.set_left_motor_PID(200, 0, 5, 0, 0, -10000, 10000)
    controller.set_right_motor_PID(200, 0, 5, 0, 0, -10000, 10000)

    time.sleep(3)

    controller.home_table(current_threshold=10, speed=20)
    controller.log_current_state()

    time.sleep(10000)

    start_time = time.time()


    while(1):
        controller.log_current_state()
        current_time = time.time() - start_time
        x_pos = travel_distance * (math.cos(angular_frequency * current_time)-1)
        y_pos = travel_distance * math.sin(angular_frequency * current_time)

        controller.command_position(0, y_pos)
        time.sleep(0.005)
        print(controller.read_motor_controller_voltages())