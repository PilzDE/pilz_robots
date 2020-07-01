# Overview
This package provides a joint trajectory controller which can be put into a holding mode through a service call.
It contains a specialized version of `ros_controllers::JointTrajectoryController` providing the holding mode functionality.
A controlled stop using a hold trajectory is performed thus stopping the manipulator without the mechanical stress of a hard brake.

**Topic interface deprecated:**
Due to safety reasons, the command interface of the `PilzJointTrajectoryController` is deactivated. For more information
see issue [#493](https://github.com/ros-controls/ros_controllers/issues/493) on ros-controls/ros_controllers.

## Speed monitoring
The controller can perform a speed monitoring of the robot links in cartesian space. If one or more robot links move faster than 250 mm/s, a controlled stop is executed. This is a requirement in operation mode T1 from DIN EN ISO 10218-1. See [prbt_hardware_support](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/prbt_hardware_support).

The speed monitoring is activated by default.

Additionally the controller limits the joint acceleration of the performed trajectories. In the file [manipulator_controller.yaml](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/prbt_support/config/manipulator_controller.yaml) these limits can be adjusted.

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
