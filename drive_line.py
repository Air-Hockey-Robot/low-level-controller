import time
import math
from roboclaw_3 import Roboclaw
from log_data import LogData
import AirHockeyTable



#Inputs
pulley_radius = 0.035 #m MEASURE THIS
start_time = time.time()
period = 2 #s
angular_frequency = 2 * math.pi / period #rad/s
circle_radius = 0.1 #m

#Zero encoders
rc_l.SetEncM1(l_address, 0)
rc_r.SetEncM1(r_address, 0)

#Set PID
rc_l.SetM1PositionPID(l_address, 150, 0, 0, 2090, 0, 0, 10000)
rc_r.SetM1PositionPID(r_address, 150, 0, 0, 2122, 0, 0, 10000)

#Drive to starting point

while(1):
	current_time = time.time() - start_time
	x_pos = 0
	y_pos = circle_radius * math.sin(angular_frequency * current_time)

	theta_l_pos = int(round(2048/(2*math.pi) * (x_pos + y_pos) / pulley_radius))
	theta_r_pos = int(round(2048/(2*math.pi) * (x_pos - y_pos) / pulley_radius))
	rc_l.SpeedAccelDeccelPositionM1(l_address, 209062, 209062, 209062, theta_l_pos, 0)
	rc_r.SpeedAccelDeccelPositionM1(r_address, 209062, 209062, 209062, theta_r_pos, 0)

	print('Left:', theta_l_pos - rc_l.ReadEncM1(l_address)[1], 'Right:', theta_r_pos - rc_r.ReadEncM1(r_address)[1])



	