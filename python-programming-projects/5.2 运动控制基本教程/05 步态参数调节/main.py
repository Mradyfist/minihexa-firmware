import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control


# create the robot object
robot = Hiwonder.Robot()

# forward motion
robot.go([0, 2, 0], 3, 600)
time.sleep(4)

# forward motion
robot.go([0, 2, 0], 2, 1000)
time.sleep(4)

# backward motion
robot.go([0, -2, 0], 3, 600)
time.sleep(4)

# backward motion
robot.go([0, -2, 0], 2, 1000)
time.sleep(4)

# rotate clockwise
robot.go([0, 0, -2], 3, 600)
time.sleep(4)

# rotate clockwise
robot.go([0, 0, -2], 2, 1000)
time.sleep(4)

# reset the robot to its initial state
robot.reset()


