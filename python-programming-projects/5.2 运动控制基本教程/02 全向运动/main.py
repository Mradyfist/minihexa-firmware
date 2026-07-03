import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control


# create the robot object
robot = Hiwonder.Robot()

# move along the +Y axis (forward)
robot.go([0, 3, 0], 3, 1000)
time.sleep(3)

# move along the +X and +Y axes (upper-right 45-degree direction)
robot.go([2, 2, 0], 3, 1000)
time.sleep(3)

# move along the +X axis (rightward)
robot.go([3, 0, 0], 3, 1000)
time.sleep(3)

# move along the +X and -Y axes (lower-right 45-degree direction)
robot.go([2, -2, 0], 3, 1000)
time.sleep(3)

# move along the -Y axis (backward)
robot.go([0, -3, 0], 3, 1000)
time.sleep(3)

# move along the -X and -Y axes (lower-left 45-degree direction)
robot.go([-2, -2, 0], 3, 1000)
time.sleep(3)

# move along the -X axis (leftward)
robot.go([-3, 0, 0], 3, 1000)
time.sleep(3)

# move along the -X and +Y axes (upper-left 45-degree direction)
robot.go([-2, 2, 0], 3, 1000)
time.sleep(4)  # wait 4 seconds



