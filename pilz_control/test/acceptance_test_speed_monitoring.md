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

# Acceptance Test Speed Monitoring using the real robot

## Prerequisites
  - Properly connect and startup the robot and power cabinet containing the PSS4000.
  - Make sure an emergency stop is within reach.

## Start in T1 mode

### Test Sequence

  1. Press the acknowledge button. Make sure the green light on the robot is on and mode T1 is enabled. Run
  ```
  roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_command_planner
  ```
  and
  ```
  roslaunch pilz_control joint_states_speed_observer.launch
  ```
  2. Execute `rosrun pilz_control acceptance_test_speed_monitoring.py`.
  3. Perform some movements via RViz. Choose different velocity scales.
  4. Switch into AUTO mode. Make sure the drives are enabled again by pressing the acknowledge button. Repeat 2. and 3.
  5. Switch back into T1 mode. Make sure the drives are enabled again by pressing the acknowledge button. Repeat 2. and 3.

### Expected Results
  1. The robot starts properly and is moveable via Rviz.
  2. Check the output on the console. The maxmimum frame speed should **not** have exceeded the limit of `0.25m/s`.
  3. Same as 2.
  4. Check the output on the console. The maximum frame speed should have exceeded the limit of `0.25m/s`. The robot movements should **not** have been interrupted.
  5. Same as 2.

## Start in AUTO mode

### Test Sequence

  1. Press the acknowledge button. Make sure the green light on the robot is on and mode AUTO is enabled. Run
  ```
  roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_command_planner
  ```
  and
  ```
  roslaunch pilz_control joint_states_speed_observer.launch
  ```
  2. Execute `rosrun pilz_control acceptance_test_speed_monitoring.py`.
  3. Perform some movements via RViz. Choose different velocity scales.

### Expected Results
  1. The robot starts properly and is moveable via Rviz.
  2. Check the output on the console. The maximum frame speed should have exceeded the limit of `0.25m/s`. The robot movements should **not** have been interrupted.
  3. Same as 2.
