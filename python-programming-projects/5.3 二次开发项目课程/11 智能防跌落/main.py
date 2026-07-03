import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
ir1 = Hiwonder_DEV.DEV_IR(32)  # infrared sensor 1 is connected to GPIO32
ir2 = Hiwonder_DEV.DEV_IR(18)  # infrared sensor 2 is connected to GPIO18

# main loop
while True:
    # read the state of the two infrared sensors
    # note: True means an obstacle is detected, False means no obstacle
    ir1_state = ir1.read()
    ir2_state = ir2.read()
    
    # if either sensor detects an obstacle
    if ir1_state or ir2_state:
        # backward action: back up quickly
        robot.go([0, -3, 0], 3, 1000)
        time.sleep(3)
        
        # rotate
        robot.go([0, 0, 2], 4, 1000)
        time.sleep(4)
    
    # if neither sensor detects an obstacle
    else:
        # forward action: move forward continuously
        robot.go([0, 3, 0])
    
    time.sleep(0.05)

