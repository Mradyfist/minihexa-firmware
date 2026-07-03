import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control

# initialize the robot
robot = Hiwonder.Robot()  # create the robot object

# move the body 3cm along the +X axis
robot.set_body_pose([3.0, 0.0, 0.0], 600)
time.sleep(1)  # wait 1 second

# move the body 3cm along the +Y axis
robot.set_body_pose([0.0, 3.0, 0.0], 600)
time.sleep(1)

# move the body 3cm along the -X axis
robot.set_body_pose([-3.0, 0.0, 0.0], 600)
time.sleep(1)

# move the body 3cm along the -Y axis
robot.set_body_pose([0.0, -3.0, 0.0], 600)
time.sleep(1)

# move the body 3cm along the +Z axis (raise the height)
robot.set_body_pose([0.0, 0.0, 3.0], 600)
time.sleep(1)

# move the body 2cm along the -Y axis and 1cm along the -Z axis
robot.set_body_pose([0.0, -2.0, -1.0], 600)
time.sleep(1)

# rotate the body 8 degrees about the X axis (Roll)
robot.set_body_angle([8.0, 0.0, 0.0], 600)
time.sleep(1)

# rotate the body -8 degrees about the X axis (reverse Roll)
robot.set_body_angle([-8.0, 0.0, 0.0], 600)
time.sleep(1)

# rotate the body 12 degrees about the Y axis (Pitch)
robot.set_body_angle([0.0, 12.0, 0.0], 600)
time.sleep(1)

# rotate the body -12 degrees about the Y axis (reverse Pitch)
robot.set_body_angle([0.0, -12.0, 0.0], 600)
time.sleep(1)

# rotate the body 12 degrees about the Z axis (Yaw)
robot.set_body_angle([0.0, 0.0, 12.0], 600)
time.sleep(1)

# rotate the body -12 degrees about the Z axis (reverse Yaw)
robot.set_body_angle([0.0, 0.0, -12.0], 600)
time.sleep(1)

robot.reset()



