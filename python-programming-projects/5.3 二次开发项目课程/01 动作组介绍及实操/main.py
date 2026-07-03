import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control

# create the robot object
robot = Hiwonder.Robot()

#run action group 5: act cute
robot.action_run(5)
time.sleep(4)


# reset the robot to its initial state
robot.reset()