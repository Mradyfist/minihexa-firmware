import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
asr = Hiwonder_DEV.DEV_ASR()

FORWARD_ID = 1      # "forward"
BACKWARD_ID = 2     # "backward"
LEFT_ID = 3         # "turn left"
RIGHT_ID = 4        # "turn right"
STOP_ID = 9         # "stop"
WALK_ID = 29        # "walk two steps"

# main loop
while True:
    # run speech recognition
    result = asr.getResult()
    # perform different actions based on the recognition result
    if result == FORWARD_ID:
        # move forward
        robot.go([0.0, 2.0, 0.0])
        
    elif result == BACKWARD_ID:
        # move backward
        robot.go([0.0, -2.0, 0.0])
        
    elif result == LEFT_ID:
        # turn left
        robot.go([0.0, 0.0, 2.0])
        
    elif result == RIGHT_ID:
        # turn right
        robot.go([0.0, 0.0, -2.0])
        
    elif result == STOP_ID:
        # stop
        robot.go([0.0, 0.0, 0.0])
        
    elif result == WALK_ID:
        # "walk two steps" - step forward twice
        robot.go([0.0, 2.0, 0.0],2)
        time.sleep(2)
    
    time.sleep(0.1)




