import time
import math
from roboclaw_3 import Roboclaw
from AirHockeyTable import AirHockeyTable

controller = AirHockeyTable("COM7", "COM6")

if __name__ == '__main__':
    controller.log_current_state()
    controller.zero_encoders()
    controller.log_current_state()

    # controller.command_position(0.05, 0) # move 5 cm to the left
    # controller.log_current_state()

    #Inputs
    start_time = time.time()
    period = 2 #s
    angular_frequency = 2 * math.pi / period #rad/s
    travel_distance = 0.1 #m

    #Set PID
    controller.set_left_motor_PID(30, 0, 0, 2090, 0, 0, 10000)
    controller.set_right_motor_PID(30, 0, 0, 2090, 0, 0, 10000)

    while(1):
        controller.log_current_state()
        current_time = time.time() - start_time
        x_pos = 0
        y_pos = travel_distance * math.sin(angular_frequency * current_time)
        time.sleep(0.1)

        controller.command_position(x_pos, y_pos)
        # print(controller.read_motor_currents(), controller.read_motor_currents())