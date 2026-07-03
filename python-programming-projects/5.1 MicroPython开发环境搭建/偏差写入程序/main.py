'''
  MiniHexa - deviation setup program (only needs to be set once, unless the firmware is reflashed)
'''


import Hiwonder  # import the Hiwonder robot control library
import time      # import the time module, used for delay control

# create the robot object
robot = Hiwonder.Robot()

# set the servo deviation value
robot.set_deviation(1 , 24)
robot.set_deviation(2 , 28)
robot.set_deviation(3 , 9)
robot.set_deviation(4 , -20)
robot.set_deviation(5 , -13)
robot.set_deviation(6 , -13)
robot.set_deviation(7 , 0)
robot.set_deviation(8 , -7)
robot.set_deviation(9 , 14)
robot.set_deviation(10 , 21)
robot.set_deviation(11 , -11)
robot.set_deviation(12 , 16)
robot.set_deviation(13 , 21)
robot.set_deviation(14 , 31)
robot.set_deviation(15 , -5)
robot.set_deviation(16 , 0)
robot.set_deviation(17 , -33)
robot.set_deviation(18 , -9)

time.sleep(0.5)
robot.download_deviation()
time.sleep(0.5)

print("******************************************")
print("servo deviation:")
print(robot.read_deviation())
print("******************************************")





