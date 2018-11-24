import time
from AngleMeterAlpha import AngleMeterAlpha

angleMeter = AngleMeterAlpha()
angleMeter.measure()

while True:
    print(angleMeter.getRoll())
    time.sleep(2)

