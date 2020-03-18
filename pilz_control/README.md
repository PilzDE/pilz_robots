# Overview
This package provides a joint trajectory controller which can be put into a holding mode through a service call.
It contains a specialized version of `ros_controllers::JointTrajectoryController` providing the holding mode functionality.
A controlled stop using a hold trajectory is performed thus stopping the manipulator without the mechanical stress of a hard brake.

## Speed monitoring
The controller can perform a speed monitoring of the robot links in cartesian space. If one or more robot links move faster than 250 mm/s, a controlled stop is executed. This is a requirement in operation mode T1 from DIN EN ISO 10218-1. See [prbt_hardware_support](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/prbt_hardware_support).

The speed monitoring is activated by default.

# ROS API
## Advertised service
- `is_executing` (std_srvs/Trigger)
  - Detect if the controller is currently executing a trajectory
- `monitor_cartesian_speed` (std_srvs/SetBool)
  - Activate/deactivate speed monitoring
- `hold` (std_srvs/Trigger)
  - Switch into holding mode
- `unhold` (std_srvs/Trigger)
  - Leave holding mode
