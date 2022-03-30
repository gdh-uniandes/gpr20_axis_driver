# Copyright (C) 2021 Grupo de Desminado Humanitario (Uniandes)

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Stepper motor interface for GPR-20 robot."""

# Imports the Python standard time library
import time

# Imports the GPIO library for Raspberry Pi 4
try:
    import RPi.GPIO as GPIO
    
except ImportError:
    from gpr20_axis.gpio_mock import GPIO


class StepperInterface(object):
    """Low-lever interface for stepper motor.

    This class allows moving an stepper motor whithin the requierements of
    the DRV8825 driver from Pololu. It allows to move the motor using the
    step/pin interface, so other drivers might be supported.

    Attributes:
        dir_pin (int): direction pin for stepper motor driver. Must be defined
            according to Raspberry Pi 4 board numbering.
        step_pin (int): step pin for stepper motor driver. Must be defined
            according to Raspberry Pi 4 board numbering.
    """

    def __init__(self, dir_pin, step_pin):
        """Initialize class with the direction and step pins.

        Args:
            dir_pin (int): pin number for connecting the direction signal of
                stepper motor driver. Must be defined according to Raspberry
                Pi 4 board numbering.
            step_pin (int): pin number for connecting the step signal of
                stepper motor driver. Must be defined according to Raspberry
                Pi 4 board numbering.
        """
        # Stores the step and direction pins as class attributes
        self._dir_pin = dir_pin
        self._step_pin = step_pin

        # Sets the board mode for GPIO handling
        GPIO.setmode(GPIO.BOARD)

        # Sets up the pins as outputs
        GPIO.setup(self._dir_pin, GPIO.OUT)
        GPIO.setup(self._step_pin, GPIO.OUT)

    def step(self, direction, pause_val):
        """Perform a step in the given direction and pause time.

        Args:
            direction (bool): sets the moving direction for step. 'True'
                for positive axis direction or CCW rotation. 'False'
                otherwise.
            pause_val (float): sets the pause time value for step. Pause
                value must be defined in seconds.
        """
        # Sets the direction pin
        GPIO.output(self._dir_pin, not direction)

        # Drives the output pin to high
        GPIO.output(self._step_pin, True)

        # Waits for signal to be asserted
        time.sleep(pause_val)

        # Drives the output pin to low
        GPIO.output(self._step_pin, False)

        # Waits for signal to be asserted
        time.sleep(pause_val)

    @property
    def dir_pin(self):
        """Return the direction pin for the motor interface."""
        return self._dir_pin

    @property
    def step_pin(self):
        """Return the step pin for the motor interface."""
        return self._step_pin
