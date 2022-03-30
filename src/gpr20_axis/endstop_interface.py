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

"""Endstop sensor interface class for the GPR-20 robot."""

# Imports GPIO library
try:
    import RPi.GPIO as GPIO
except ImportError:
    from gpr20_axis.gpio_mock import GPIO


class EndstopInterface(object):
    """Low-level interface for endstop sensor.

    This class allows to connect and sample the endstop sensor from the GPR-20
    robot. The class uses the RPi library for using the GPIO pins from the
    Raspberry Pi board.

    Attributes:
        sensor_pin (int): pin number for connecting the data wire of endstop
            sensor. Must be defined according to board pin number.
    """

    def __init__(self, sensor_pin):
        """Intializes the endstop interface with a given pin number.

        Args:
          sensor_pin (int): pin number for endstop sensor data wire. Must be
            defined according to board pin number.
        """
        # Stores the sensor pin number as class attribute
        self._sensor_pin = sensor_pin

        # Sets the board mode for GPIO handling
        GPIO.setmode(GPIO.BOARD)

        # Sets up the pins as input
        GPIO.setup(self._sensor_pin, GPIO.IN)

    def sample(self):
        """Sample the endstop sensor value.

        Returns:
          bool: the logical value for endstop sensor. 'True' for pressed,
            'False' otherwise.
        """
        # Returns logic invertion of sampled pin value
        return not GPIO.input(self._sensor_pin)

    @property
    def sensor_pin(self):
        """Return the sensor pin for the endstop sensor interface."""
        return self._sensor_pin
