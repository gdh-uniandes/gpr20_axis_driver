# GPR-20 Axis ROS Package
This is a ROS package used to handle an axis of the GPR-20 robot. This package provides the low level driver for the stepper motors and an interface for the endstop sensor. The axis is driven using an action (AxisAction) that moves the axis into a given coordinate. 

## Axis Action
The purpose of the axis action is to provide an interface for other packages to move the axis into a given position without blocking the entire system execution. The action is defined as follows:

- **Goal:** is defined as the target coordinate of the axis. If goal is set beyond the axis limits then it is set as aborted.
- **Feedback:** is defined as the current coordinate of the axis.
- **Result:** tells when the axis has reached the target coordinate.

The axis action is defined on the GPR20_msgs package.
