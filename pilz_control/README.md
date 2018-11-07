# Overview
This package provides a joint trajectory controller which can be put into a holding mode through a service call.
It contains a specialized version of `ros_controllers::JointTrajectoryController` providing the holding mode functionality.
A controlled stop using a hold trajectory is performed thus stopping the manipulator without the mechanical stress of a hard brake.

# ROS API
## Advertised service
- `hold` (std_srvs/Trigger)
  - Switch into holding mode
- `unhold` (std_srvs/Trieer)
  - Leave holding mode
