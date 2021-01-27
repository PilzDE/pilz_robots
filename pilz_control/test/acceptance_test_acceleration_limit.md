<!--
Copyright © 2020 Pilz GmbH & Co. KG

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

-->

# Acceptance Test Acceleration Limit using the real robot

## Prerequisites
  - Properly connect and startup the robot and power cabinet containing the PSS4000.
  - Make sure an emergency stop is within reach.
  - **Be careful, the robot might crash into the table.**

### Test Sequence

  1. Press the acknowledge button. Make sure the green light on the robot is on and switch into T1 mode (if not active yet). Run
  ```
  roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_industrial_motion_planner
  ```
  and execute
  ```
  rosrun pilz_control acceptance_test_acceleration_limit.py
  ```
  2. Switch into AUTO mode. Make sure the drives are enabled again by pressing RESET and execute
  ```
  rosrun pilz_control acceptance_test_acceleration_limit.py
  ```
  3. Use RViz to plan a movement with velocity- and acceleration scaling 1.0. Check if it can be planned successfully. Execute it.

### Expected Results
  1. All tests pass
  2. All tests pass
  3. Movement is executed without errors.

## Running the Test Without Real Hardware

  This can be useful but is not sufficient for the acceptance of the feature. The `robot_mock` that is used here does not represent the PRBT in any way. Simply replace the `roslaunch`-command by
  ```
  roslaunch pilz_control robot_mock.launch
  ```
  and proceed as described above. In order to simulate switching into AUTO mode, you can call
  ```
  rosservice call /controller_ns/test_joint_trajectory_controller/monitor_cartesian_speed "data: false"
  ```
