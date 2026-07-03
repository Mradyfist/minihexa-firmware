import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
cam = Hiwonder_DEV.DEV_ESP32S3Cam()

position = [0, 0, 0]  # [x, y, z] unit: cm
initial_z = position[2]  # save the initial height

while True:
  rec = cam.read_face()
  if rec:
    if rec[2] > 0:
      # pitch +10 degrees
      robot.set_body_angle([0, 10, 0], 300)
      time.sleep_ms(300)
      # pitch -10 degrees
      robot.set_body_angle([0, -10, 0], 300)
      time.sleep_ms(300)
      # pitch +10 degrees
      robot.set_body_angle([0, 10, 0], 300)
      time.sleep_ms(300)
      # pitch -10 degrees
      robot.set_body_angle([0, -10, 0], 300)
      time.sleep_ms(300)
      # return to the level pose
      robot.set_body_angle([0, 0, 0], 300)
      time.sleep_ms(300)
      # move 2cm to the right
      robot.set_body_pose([2.0, 0, 0], 200)
      time.sleep_ms(200)
      # move 2cm to the left
      robot.set_body_pose([-2.0, 0, 0], 200)
      time.sleep_ms(200)
      # move 2cm to the right
      robot.set_body_pose([2.0, 0, 0], 200)
      time.sleep_ms(200)
      # move 2cm to the left
      robot.set_body_pose([-2.0, 0, 0], 200)
      time.sleep_ms(200)
      # return to the center position
      robot.set_body_pose([0, 0, 0], 200)
      time.sleep_ms(200)
  time.sleep(0.04)



