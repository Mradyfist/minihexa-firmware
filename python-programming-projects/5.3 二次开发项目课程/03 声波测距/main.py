import Hiwonder
import Hiwonder_DEV
import time

# create the robot object and the ultrasonic sensor object
robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()

# initialize the RGB light to white
sonar.setRGB(0, 255, 255, 255)  # arg 1: light index (0 means control both lights at once)
                                # args 2-4: RGB values (255,255,255 is white)

def map_value(x, in_min, in_max, out_min, out_max):
    """map a value from one range to another"""
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# main loop
while True:
    # read the ultrasonic sensor distance (unit: cm)
    distance = sonar.getDistance()
    
    # set the RGB color based on the distance
    if distance > 0 and distance <= 8:
        # breathing-light mode (red) - the actual implementation needs a breathing effect
        r, g, b = 255, 0, 0
    
    elif distance > 8 and distance <= 18:
        # red gradient (distance 80-180 mapped to 255-0)
        s = map_value(distance, 8, 18, 0, 255)
        r, g, b = 255 - s, 0, 0
    
    elif distance > 18 and distance <= 32:
        # blue gradient (distance 180-320 mapped to 0-255)
        s = map_value(distance, 18, 32, 0, 255)
        r, g, b = 0, 0, s
    
    elif distance > 32 and distance <= 50:
        # green gradient (distance 320-500 mapped to 0-255)
        s = map_value(distance, 32, 50, 0, 255)
        r, g, b = 0, s, 255 - s
    
    else:  # distance > 500
        # green
        r, g, b = 0, 255, 0
    
    # set the RGB lights on the ultrasonic sensor
    sonar.setRGB(0, r, g, b)  # set both lights at once
    print("Distance:", distance, "mm")
    time.sleep_ms(100)



