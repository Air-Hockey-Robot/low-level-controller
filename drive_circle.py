import time
import math
from roboclaw_3 import Roboclaw
from log_data import LogData

#Windows comport name
rc_r = Roboclaw("COM5",460800)
rc_l = Roboclaw("COM4",460800)
#Linux comport name
#rc = Roboclaw("/dev/ttyACM0",115200)

rc_l.Open()
rc_r.Open()

l_address = 0x80
r_address = 0x85

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
x_start_pos = circle_radius
y_start_pos = 0

theta_l_start_pos = int(round(2048/(2*math.pi) * (x_start_pos + y_start_pos) / pulley_radius))
theta_r_start_pos = int(round(2048/(2*math.pi) * (x_start_pos - y_start_pos) / pulley_radius))

print(rc_l.ReadEncM1(l_address))
print(rc_r.ReadEncM1(r_address))

rc_l.SpeedAccelDeccelPositionM1(l_address, 209062, 209062, 209062, theta_l_start_pos, 0)
rc_r.SpeedAccelDeccelPositionM1(r_address, 209062, 209062, 209062, theta_r_start_pos, 0)
time.sleep(5)


while(1):
	current_time = time.time() - start_time
	x_pos = circle_radius * math.cos(angular_frequency * current_time)
	y_pos = circle_radius * math.sin(angular_frequency * current_time)

	theta_l_pos = int(round(2048/(2*math.pi) * (x_pos + y_pos) / pulley_radius))
	theta_r_pos = int(round(2048/(2*math.pi) * (x_pos - y_pos) / pulley_radius))
	rc_l.SpeedAccelDeccelPositionM1(l_address, 209062, 209062, 209062, theta_l_pos, 0)
	rc_r.SpeedAccelDeccelPositionM1(r_address, 209062, 209062, 209062, theta_r_pos, 0)

	print('Left:', theta_l_pos - rc_l.ReadEncM1(l_address)[1], 'Right:', theta_r_pos - rc_r.ReadEncM1(r_address)[1])



	