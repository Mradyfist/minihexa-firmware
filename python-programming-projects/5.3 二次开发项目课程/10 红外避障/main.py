import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
ir1 = Hiwonder_DEV.DEV_IR(32)
ir2 = Hiwonder_DEV.DEV_IR(18)

# main loop
while True:
    # read the state of the two infrared sensors
    # note: True means an obstacle is detected, False means no obstacle
    ir1_state = ir1.read()
    ir2_state = ir2.read()
    
    if ir1_state and ir2_state:  # both sensors detect an obstacle
        # move backward for 2 seconds
        robot.go([0, -2, 0], 2, 800)
        time.sleep(2)  # wait 2 seconds
        
        # rotate clockwise
        robot.go([0, 0, 2], 2, 800)
        time.sleep(2)  # wait 2 seconds
    
    elif ir1_state and not ir2_state:  # only sensor 1 detects an obstacle
        # rotate clockwise
        robot.go([0, 0, 2], 2, 800)
        time.sleep(2)  # wait 2 seconds
    
    elif not ir1_state and ir2_state:  # only sensor 2 detects an obstacle
        # rotate counterclockwise
        robot.go([0, 0, -2], 2, 800)
        time.sleep(2)  # wait 2 seconds
    
    else:  # neither sensor detects an obstacle
        # move forward (continuous motion)
        robot.go([0, 2, 0], -1, 800)
    time.sleep(0.1)
