from __future__ import print_function
import time
import pigpio
import argparse
from dual_g2_hpmd_rpi import motors, MAX_SPEED

parser = argparse.ArgumentParser(description='set speed and ramp period parameter')
parser.add_argument('speed', metavar='N', type=int, help='speed (-480, 480)')
parser.add_argument('period', metavar='t', type=float, help='period (seconds)')
parser.add_argument('motor', metavar='M', type=int, help='motor (1,2,3)')
args = parser.parse_args()

_max_speed = 480

_pin_M3DIR = 27
_pin_M3EN = 17
_pin_M3PWM = 20
_pin_M3FLT = 7

#_pin_ENC1 = 
#_pin_ENC2 = 

TARGET = 45
KP = 0.02
SAMPLETIME = 1

posi = 0
tprev = 0
eprev = 0
eintegral = 0

_pi = pigpio.pi()
if not _pi.connected:
    raise IOError("Can't connect to pigpio")

# Callback that increments on either rising or falling edge to count encoder steps
# Call cb1.tally() to get count and cb1.reset_tally() to reset count 
#cb1 = _pi.callback(_pin_ENC1, EITHER_EDGE)
#cb2 = _pi.callback(_pin_ENC2, EITHER_EDGE)

class Actuator(object):
    MAX_SPEED = _max_speed

    def __init__(self, pwm_pin, dir_pin, en_pin, flt_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.flt_pin = flt_pin

        _pi.set_pull_up_down(flt_pin, pigpio.PUD_UP) # make sure FLT is pulled up
        _pi.write(en_pin, 1) # enable driver by default

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        _pi.write(self.dir_pin, dir_value)
        #_pi.hardware_PWM(self.pwm_pin, 20000, int(speed * 6250 / 3));
        _pi.set_PWM_range(self.pwm_pin, 480)
        _pi.set_PWM_frequency(self.pwm_pin, 20000)
        _pi.set_PWM_dutycycle(self.pwm_pin, speed)
          # 20 kHz PWM, duty cycle in range 0-1000000 as expected by pigpio

    def enable(self):
        _pi.write(self.en_pin, 1)

    def disable(self):
        _pi.write(self.en_pin, 0)

    def getFault(self):
        return not _pi.read(self.flt_pin)

    def forceStop(self):
        global _pi
        _pi.stop()
        _pi = pigpio.pi()
        self.setSpeed(0)

motor3 = Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT)


# Define a custom exception to raise if a fault is detected.
class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)

# class Encoder(object):
#     def __init__(self, pin):
#         self._value = 0
#         encoder = 
#     def reset(self):
#         self._value = 0
#     def _increment(self):
#         self._value += 1
# @property
# def value(self):
#     return self._value


try:
    #while True:
    #    e1_error = TARGET - cb1.tally()
    #    motors.motor1.setSpeed(e1_error * KP)
    #    cb1.reset_tally()
    #    time.sleep(SAMPLETIME)

        #e2_error = TARGET - cb
    if (args.motor == 1):
        motors.motor1.setSpeed(args.speed)
        time.sleep(args.period)
        motors.motor1.setSpeed(0)
    elif (args.motor == 2):
        motors.motor2.setSpeed(args.speed)
        time.sleep(args.period)
        motors.motor2.setSpeed(0)
    elif (args.motor == 3):
        motor3.setSpeed(args.speed)
        time.sleep(args.period)
        motor3.setSpeed(0)
    # if args.speed > 0:
    #     for i in range(0,args.speed,1):
    #         motors.motor1.setSpeed(i)
    #         motors.motor2.setSpeed(i)
    #         raiseIfFault()
    #         time.sleep(args.period)
    #     for i in range(args.speed,-1,-1):
    #         motors.motor1.setSpeed(i)
    #         motors.motor2.setSpeed(i)
    #         raiseIfFault()
    #         time.sleep(args.period)
    # elif args.speed < 0:
    # 	for i in range(0, args.speed, -1):
    # 		motors.motor1.setSpeed(i)
    # 		motors.motor2.setSpeed(i)
    # 		raiseIfFault()
    # 		time.sleep(args.period)
    # 	for i in range(args.speed, 1, 1):
    # 		motors.motor1.setSpeed(i)
    # 		motors.motor2.setSpeed(i)
    # 		raiseIfFault()
    # 		time.sleep(args.period)
    #time.sleep(2.5)

    #motors.motor1.setSpeed(0)
    #motors.motor2.setSpeed(0)



except DriverFault as e:
    print("Driver %s fault!" % e.driver_num)


finally:
  # Stop the motors, even if there is an exception
  # or the user presses Ctrl+C to kill the process.
    motors.forceStop()
    motor3.forceStop()
