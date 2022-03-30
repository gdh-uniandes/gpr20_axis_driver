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

"""Axis driver for the GPR-20 robot."""

from gpr20_axis.stepper_interface import StepperInterface
from gpr20_axis.endstop_interface import EndstopInterface


class AxisDriver(object):
    """High-level axis driver for the GPR-20 robot.

    This class introduces major and utility methods for managing a single axis
    of the GPR-20 robot. The driver is able to both manage linear and
    rotational axes.

    Attributes:
        step_pin (int): step pin number for the axis motor. It must lie within
            1 and 40. Can not be equal to any other pin value (direction and
            sensor).
        dir_pin (int): direction pin number for the axis motor. It must lie
            within 1 and 40. Con not be equal to any other pin value (step and
            sensor).
        stepper_interface (StepperInterface): stepper motor hardware interface
            for the axis driver.
        sensor_pin (int): sensor pin number for the endstop sensor data line.
            It must lie within 1 and 40. Con not be equal to any other pin
            value (step and dir).
        endstop_interface (EndstopInterface): endstop sensor hardware interface
            for the axis driver.
        step_size (float): length of the step size for the axis. Must be
            greater than zero.
        min_step (float): minimum duration of a step. Defines the axis maximum
            speed. Must be greater than zero and can not be greater than the
            step maximum duration.
        max_step (float): maximum duration of a step. Defines the axis minimum
            speed. Must be greater than zero and can not be lower than the
            step minimum duration.
        delta_step (float): delta time for modifiying the step time. Defines
            how the step time changes during acceleration and breaking. Only
            integer delta steps are allowed to cover the difference between
            minimum and maximum step durations.
        step_intervals (int): available speed intervals for the axis. Defines
            how many different speed values the axis can take. It is calculated
            from the minimum and maximum step durations and the delta time.
        min_coord (float): minimum coordinate value that the axis can take.
            The current coordinate attribute intializes to this value after
            homing sequence.
        max_coord (float): maximum coordinate value that the axis can take.
        current_coord (float): current coordinate value of the axis.
        axis_type (str): defines the axis type.
        homing_done (bool): tells if axis executed homing routine or not.
        busy (bool): tells if axis is busy.
    """

    def __init__(self, pin_values, step_values, coord_values, axis_type):
        """Initialize the axis driver class.

        The constructor initializes every attribute for the axis driver. The
        current coordinate attribute is the only attribute that is not set on
        class initialization. This attribute is only set when the homing
        sequence is executed.

        Args:
            pin_values (list): contains values for the direction, step and
                sensor pins for the axis. Must have a length of three items.
                Sensor pin value is discarded if axis is of type rotational.
            step_values (list): contains values for the step size, minumum and
                maximum step durations and the step delta time value. Must
                have a length of four items.
            coord_values (list): contains values for the minumum and maximum
                axis coordinates for the axis. Must hace a length of two
                items.
            axis_type (str): sets the type of the axis. It can be either 'LIN'
                for linear or 'ROT' for rotational. Default value sets driver
                to linear type ('LIN').

        Raises:
            AxisException: an exception occured when intializating the axis
                driver. Possible exceptions are that parameters list are not
                of required length and/or hardware pins are repeated.
        """
        # Initializes the motor interface pins values
        self._dir_pin, self._step_pin, = pin_values[0], pin_values[1]

        # Checks that stepper pins are different or raises exception
        if self._step_pin == self._dir_pin:
            raise AxisException(
                "Pins number for step and "
                "direction must be different")

        # Initializes the motor interface
        self._stepper_interface = StepperInterface(
            self._dir_pin,
            self._step_pin)

        # Stores the attributes for step values
        self._step_size = step_values[0]
        self._min_step, self._max_step = step_values[1], step_values[2]
        self._delta_step = step_values[3]

        # Calculate the available step intervals
        step_time_delta = self._max_step - self._min_step
        self._step_intervals = int(step_time_delta / self._delta_step) + 1

        # Stores the attributes for coordinates
        self._min_coord, self._max_coord = coord_values[0], coord_values[1]

        # Initializes the type attribute
        self._axis_type = axis_type

        # Initializes the endstop interface if axis is linear
        if self._axis_type == "LIN":

            # Stores the sensor pin
            self._sensor_pin = pin_values[2]

            # Checks that sensor pin is not repeated
            if self._dir_pin == self._sensor_pin:
                raise AxisException(
                    "Direction pin number is equal to sensor pin.")

            elif self._step_pin == self._sensor_pin:
                raise AxisException("Step pin number is equal to sensor pin.")

            # Initializes the endstop interface
            self._endstop_interface = EndstopInterface(self._sensor_pin)

        # Sets sensor pin and endstop interface to none if axis is rotational
        elif self._axis_type == "ROT":

            # Sets values for both sensor pin and endstop interface
            self._sensor_pin, self._endstop_interface = None, None

        # Initializes the current coordinate attribute
        self._current_coord = -1.0

        # Initializes the homing done attribute
        self._homing_done = False

        # Intializes the busy attribute
        self.__busy = False

    def homing(self):
        """Perform the homing sequence for the axis.

        If axis is rotational, calling the homing  method will initialize the
        current coordinate to the minimum coordinate value. If axis is linear
        it will execute the homing sequence.

        The homing sequence for the linear axis consists of executing steps on
        the axis' negative direction until the endstop sensor is asserted.
        There is a maximum step number that is calculated from the axis limits
        and the step size. If the maximum steps are achieved, an exception
        raises to warn the user about a potential hardware issue.

        Raises:
            AxisException: if linear homing reaches the maximum calculated
                steps for the axis.

        """
        # Checks that axis type is rotational
        if self._axis_type == 'ROT':

            # Initializes the current coordinate to minimum coordinate
            self._current_coord = self._min_coord

            # Set homing attribute
            self._homing_done = True

        # Executes homing sequence for linear axis
        else:

            # Calculates the maximum steps that can be performed on axis
            max_steps = (self._max_coord - self._min_coord) / self._step_size

            # Creates a stop variable for sequence
            reached_sensor = False

            # Creates a counter
            executed_steps = 0

            # Homing sequence runs on a loop until
            while not reached_sensor:

                # Executes an step on negative direction at minimum speed
                self._stepper_interface.step(False, self._max_step)

                # Increments the step count
                executed_steps += 1

                # Checks sensor value
                reached_sensor = self._endstop_interface.sample()

                # Checks if maximum step have been achieved
                if executed_steps == max_steps:
                    raise AxisException(
                        "Maximum steps for axis have been achieved")

            # Sets minimum coordinate value
            self._current_coord = self._min_coord

            # Sets the homing attribute to true
            self._homing_done = True

    def move_to_coordinate(self, target_coord):
        """Move axis positioner to a target coordinate.

        Args:
            target_coord (float): target cooordinate for the axis positioner.
        """
        # Checks that axis is not busy
        self._check_availability()

        # Sets axis as busy
        self.__busy = True

        # Checks that axis has performed homing
        self._check_homing()

        # Checks that axis is within valid coordinates
        self._check_valid_target(target_coord)

        # Calculate the distance delta to target coordinate
        distance_delta = self._calculate_delta_distance(target_coord)

        # Get the movement direction
        move_direction = self._calculate_direction(distance_delta)

        # Get the required steps to reach target
        n_steps = self._calculate_steps(distance_delta)

        # Get the step time list
        steps_list = self._create_steps_list(n_steps)

        # Executes the axis movement
        for pause_val in steps_list:

            # Moves the axis at the defined speed
            self._stepper_interface.step(move_direction, pause_val)

            # Updates the current coordinate value
            if move_direction:
                self._current_coord += self._step_size
            else:
                self._current_coord -= self._step_size

        # Releases the axis
        self.__busy = False

    def _check_availability(self):
        """Checks if axis is available."""
        if self.__busy:
            raise AxisException("Axis is busy!")

    def _check_homing(self):
        """Checks if axis routine was performed."""
        if not self._homing_done:
            self.__busy = False
            raise AxisException("Homing has not been performed!")

    def _check_valid_target(self, target_coord):
        """Check that the target coordinate is valid.

        Args:
            target_coord (float): target coordinate for the axis.

        Raises:
            AxisException: if target coordinate does not lie within the
                defined boundaries for axis.

        """
        if target_coord > self._max_coord:
            self.__busy = False            
            raise AxisException(
                "Target coordinate is above maximum coordinate for axis!"
                )
        elif target_coord < self._min_coord:
            self.__busy = False
            raise AxisException(
                "Targte coordinate is below minimum coordinate for axis!"
                )

    def _calculate_delta_distance(self, target_coord):
        """Calculate the distance delta to target coordinate.

        Args:
            target_coord (float): target coordinate for the axis.
        """
        return self.current_coord - target_coord

    @staticmethod
    def _calculate_direction(distance_delta):
        """Calculate the axis direction to reach a target coordinate.

        Args:
            target_coord(float): target coordinate for the axis.

        Returns:
            bool: Axis direction as boolean value. True for positive
                direction, False otherwise.
        """
        return True if distance_delta < 0 else False

    def _calculate_steps(self, distance_delta):
        """Calculate the steps number to reach a target coordinate.

        Args:
            target_coord (float): target coordinate for the axis.

        Returns
            int: required steps to reach the target coordinate.
        """
        return int(abs(distance_delta) / self._step_size)

    def _create_steps_list(self, n_steps):
        """Calculate the duration of each step to move axis.

        Args:
            n_steps (int): step count to reach a target coordinate.

        Returns:
            list: with time pauses for each step on axis movement.
        """
        # Creates the steps times list
        steps_list = []

        # Moves at constant low speed if can not accelerate
        if n_steps < 2 * self._step_intervals and n_steps > 0:
            steps_list = [self._max_step for i in range(n_steps)]

        # Acceleration and deceleration with no constant speed
        elif n_steps == 2 * self._step_intervals:

            # Defines the acceleration interval
            for i in range(self._step_intervals):
                steps_list.append(self._max_step - i * self._delta_step)

            # Defines the deceleration interval
            for i in range(self._step_intervals):
                steps_list.append(self._min_step + i * self._delta_step)

        # Acceleration and deceleration with constant speed
        elif n_steps > 2 * self._step_intervals:

            # Calculates how many constant speed steps are required
            const_steps = n_steps - (2 * self._step_intervals)

            # Defines the acceleration interval
            for i in range(self._step_intervals):
                steps_list.append(self._max_step - i * self._delta_step)

            # Defines the constant speed interval
            for i in range(const_steps):
                steps_list.append(self._min_step)

            # Defines the deceleration interval
            for i in range(self._step_intervals):
                steps_list.append(self._min_step + i * self._delta_step)

        # Returns the steps time list
        return steps_list

    @property
    def current_coord(self):
        """Return the current axis coordinate."""
        return self._current_coord

    @property
    def homing_done(self):
        """Return boolean flag with homing status."""
        return self._homing_done


class AxisException(Exception):
    """Defines a custom exception for the axis.

    Attributes:
        message (str): text message that will be displayed to user.
    """

    def __init__(self, message):
        """Initilize the custom axis exception.

        Args:
            message (str): message that will be displayed for the exception.
        """
        # Stores the message
        self._message = message

        # Initializes parent class
        super(AxisException, self).__init__(self._message)

    def __str__(self):
        """Return the message."""
        return self._message

    @property
    def message(self):
        """Returns the message attribute."""
        return self._message
