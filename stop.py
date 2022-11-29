import time
from roboclaw_3 import Roboclaw

#Windows comport name
rc_r = Roboclaw("COM5",460800)
rc_l = Roboclaw("COM4",460800)
#Linux comport name
#rc = Roboclaw("/dev/ttyACM0",115200)

rc_l.Open()
rc_r.Open()

l_address = 0x80
r_address = 0x85

rc_l.ForwardM1(l_address,0)
rc_r.BackwardM1(r_address,0)