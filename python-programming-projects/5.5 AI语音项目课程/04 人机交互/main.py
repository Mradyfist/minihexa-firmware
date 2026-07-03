import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()
asr = Hiwonder_DEV.DEV_ASR()

def perform_dance():
    global robot
    # part one: pitch swing (pitch oscillation)
    # pitch +10 degrees (body leans forward)
    robot.set_body_angle([0.0, 10.0, 0.0], 300)
    time.sleep_ms(600)  # wait for the action to complete + an extra delay
    
    # pitch -10 degrees (body leans back)
    robot.set_body_angle([0.0, -10.0, 0.0], 300)
    time.sleep_ms(600)
    
    # pitch +10 degrees (body leans forward)
    robot.set_body_angle([0.0, 10.0, 0.0], 300)
    time.sleep_ms(600)
    
    # pitch -10 degrees (body leans back)
    robot.set_body_angle([0.0, -10.0, 0.0], 300)
    time.sleep_ms(600)
    
    # return to the level pose
    robot.set_body_angle([0.0, 0.0, 0.0], 300)
    time.sleep_ms(600)
    
    # part two: horizontal swing (movement along the x axis)
    # move 2cm to the right
    robot.set_body_pose([2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # move 2cm to the left
    robot.set_body_pose([-2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # move 2cm to the right
    robot.set_body_pose([2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # move 2cm to the left
    robot.set_body_pose([-2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # return to the center position
    robot.set_body_pose([0.0, 0.0, 0], 200)
    time.sleep_ms(400)

while True:
    result = asr.getResult()
    # perform different actions based on the recognition result
    if result == 26:
        # recognized "hello" - run action group 14
        robot.action_group_run(14)
        pass
    elif result == 27:
        # recognized "introduce yourself" - run the act-cute action
        perform_dance()
        
    elif result == 28:
        # recognized "show off" - run action group 7
        robot.action_group_run(7)
        pass
    time.sleep_ms(200)



