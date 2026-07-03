import Hiwonder
import Hiwonder_DEV
import time

# disable the low-battery alarm
Hiwonder.disableLowPowerAlarm()

# create the robot object and the ultrasonic sensor object
robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()

# initialize the RGB light to white
sonar.setRGB(0, 255, 255, 255)

# main loop
while True:
    # read the ultrasonic sensor distance (unit: cm)
    distance = sonar.getDistance()
    print(distance)
    if distance > 20:
      sonar.setRGB(0,0,250,0)
      robot.go([0,2,0])
    elif distance < 10:
      sonar.setRGB(0,250,0,0)
      robot.go([0,-2,0])
    else:
      sonar.setRGB(0,0,0,250)
      robot.go([0,0,0])
    time.sleep_ms(100)



