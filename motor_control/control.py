import time
import board
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())

for i in range(100):
    kit.stepper1.onestep(style=stepper.MICROSTEP)
    time.sleep(0.01)