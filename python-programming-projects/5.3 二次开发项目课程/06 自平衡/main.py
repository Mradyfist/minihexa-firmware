import Hiwonder
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
imu = Hiwonder.IMU()
time.sleep(1)

# enable the self-balancing function
robot.homeostasis(True)
time.sleep(3)

# disable the self-balancing function
# robot.homeostasis(False)

