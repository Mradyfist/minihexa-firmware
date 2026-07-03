import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control


# create the robot object
robot = Hiwonder.Robot()

# move forward along the +Y axis while rotating counterclockwise
robot.go([0, 3, 0.2])
time.sleep(5)

# move forward along the +Y axis while rotating clockwise
robot.go([0, 3, -0.2])
time.sleep(5)

# reset the robot to its initial state
robot.reset()


