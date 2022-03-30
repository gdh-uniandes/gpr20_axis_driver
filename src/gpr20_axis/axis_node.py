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

"""ROS node for axis driver for the GPR-20 robot."""

import rospy
import actionlib
from std_srvs.srv import Empty, EmptyResponse
from gpr20_msgs.msg import GPR20AxisStatus
from gpr20_msgs.msg import AxisAction, AxisFeedback, AxisResult

from gpr20_axis.axis_driver import AxisDriver, AxisException


class AxisNode(object):
    """Class to handle the ROS interaction for a GPR-20 robot axis.

    AxisNode intializes a ROS node and provide the actions, services and
    topics required to use the axis. Every ROS interfaction buffers the
    requests to an instance of the AxisDriver.

    Attributes:
        axis_driver (AxisDriver): driver for the axis.
    """

    def __init__(self):
        """Initialize the GPR-20 axis ROS node.

        The AxisNode class handles the ROS services, actions, topics and
        parameters.

        """
        # Initializes the GPR-20 axis node
        rospy.init_node("gpr20_axis", anonymous=False)

        # Get the pin parameters
        pin_values = [
            rospy.get_param("~dir_pin"),
            rospy.get_param("~step_pin"),
            rospy.get_param("~sensor_pin")
        ]

        # Get the step parameters
        step_values = [
            rospy.get_param("~step_size"),
            rospy.get_param("~min_step"),
            rospy.get_param("~max_step"),
            rospy.get_param("~delta_step")
        ]

        # Get the coordinate values
        coord_values = [
            rospy.get_param("~min_coord"),
            rospy.get_param("~max_coord")
        ]

        # Gets the axis type parameter
        axis_type = rospy.get_param("~type")

        # Instantiates the axis driver
        self._axis_driver = AxisDriver(
            pin_values,
            step_values,
            coord_values,
            axis_type
        )

        # Initializes the axis action server
        self._axis_action_server = actionlib.SimpleActionServer(
            'axis',
            AxisAction,
            execute_cb=self.action_execute_cb,
            auto_start=False
        )

        # Start the action server
        self._axis_action_server.start()

        # Create the feedback and result messages for action
        self._axis_action_fb = AxisFeedback()
        self._axis_action_res = AxisResult()

        # Create the current coordinate topic
        self._current_coord_pub = rospy.Publisher(
            "axis_status",
            GPR20AxisStatus,
            queue_size=100
        )

        # Creates the homing service
        rospy.Service("homing", Empty, self.homing_handler)

        # Define the rate for the node to publish data
        node_rate = rospy.Rate(100)

        # Execute the node publishers
        while not rospy.is_shutdown():

            # Publish homing status and current coordinate
            self._current_coord_pub.publish(
                self._axis_driver.homing_done,
                self._axis_driver.current_coord
            )

            # Sleep the node to match node rate
            node_rate.sleep()

    def homing_handler(self, srv):
        """Perform the homing routine for the axis.

        Args:
            srv (Empty): empty service to command axis to execute the homing
                sequence. The executed homing sequence depends on the axis
                type.
        """
        # Tries to execute the homing routine
        try:
            # Commands driver to execute homing routine
            self._axis_driver.homing()

        # Handles the error
        except AxisException as error:

            # Logs the error message
            rospy.logerr(error.message)

        # Returns the empty response
        return EmptyResponse()

    def action_execute_cb(self, goal):
        """Move axis to goal coordinate.

        Args:
            goal (AxisGoal): goal message for action. The goal includes the
                target coordinate for the axis.
        """
        # Try to execute the requested goal
        try:

            # Moves axis based on the target coordinate
            self._axis_driver.move_to_coordinate(goal.target_coordinate)

            # Sets result to tell that axis has reached the target coordinate
            self._axis_action_res.result = True

            # Publishes the suceed message
            self._axis_action_server.set_succeeded(self._axis_action_res)

        # Sets as aborted if an exception occurs
        except AxisException as error:

            # Sets results message as false
            self._axis_action_res.result = False

            # Sets goal as aborted
            self._axis_action_server.set_aborted(self._axis_action_res)

            # Logs the error
            rospy.logerr(error.message)
