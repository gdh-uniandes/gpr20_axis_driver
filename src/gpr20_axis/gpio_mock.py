
import random

class GPIO(object):
    
    BOARD = 0
    BCM = 1
    IN = 2
    OUT = 3

    def __init__(self):
        pass

    @staticmethod
    def setmode(mode):
        print("Mode set on %s." % mode)

    @staticmethod
    def setup(pin, mode):
        print("Setting pin %d on mode %d" % (pin, mode))

    @staticmethod
    def output(pin, value):
        print("Pin %d output is set to %d" % (pin, value))

    @staticmethod
    def input(pin):
        return False

