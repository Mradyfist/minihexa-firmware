import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control


# create the robot object
robot = Hiwonder.Robot()

# first rotation: low-speed rotation (1 unit of speed)
robot.go([0, 0, 1])
time.sleep(3)

# second rotation: medium-low-speed rotation (1.5 units of speed)
robot.go([0, 0, 1.5])
time.sleep(3)

# third rotation: medium-speed rotation (2 units of speed)
robot.go([0, 0, 2])
time.sleep(3)

# fourth rotation: high-speed rotation (2.5 units of speed)
robot.go([0, 0, 2.5])
time.sleep(3)

# reset the robot to its initial state
robot.reset()


