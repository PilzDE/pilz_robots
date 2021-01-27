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

  1. Press the acknowledge button. Make sure the green light on the robot is on and switch into T1 mode (if not active yet). Run
  ```
  roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_industrial_motion_planner
  ```
  and
  ```
  roslaunch pilz_control joint_states_speed_observer.launch
  ```
  2. Execute `rosrun pilz_control acceptance_test_speed_monitoring.py`.
  3. Perform movements via RViz with velocity scales `0.1` and `1.0`.
  4. Switch into AUTO mode. Make sure the drives are enabled again by pressing RESET and execute `rosrun pilz_control acceptance_test_speed_monitoring.py auto`.
  5. Perform movements via RViz with velocity scale `1.0`.
  6. Switch back into T1 mode. Make sure the drives are enabled again by pressing the acknowledge button and execute `rosrun pilz_control acceptance_test_speed_monitoring.py`.
  7. Perform movements via RViz with velocity scales `0.1` and `1.0`.

### Expected Results
  1. The robot starts properly and is moveable via Rviz.
  2. All tests pass.
  3. A movement with velocity scale of `1.0` should be interrupted by a stop. The maxmimum frame speed should **not** have exceeded the limit of `0.25m/s`.
  4. All tests pass.
  5. The robot movements should **not** have been interrupted. The maximum frame speed should have exceeded the limit of `0.25m/s`. 
  6. All tests pass.
  7. A movement with velocity scale of `1.0` should be interrupted by a stop. The maxmimum frame speed should **not** have exceeded the limit of `0.25m/s`.

## Running the Test Without Real Hardware

  This can be useful but is not sufficient for the acceptance of the feature. The `robot_mock` that is used here does not represent the PRBT in any way. Simply replace the `roslaunch`-commands by
  ```
  roslaunch pilz_control robot_mock.launch
  ```
  and
  ```
  roslaunch pilz_control joint_states_speed_observer.launch robot_name:=robot_mock
  ```
  Then proceed as described above. In order to simulate switching into AUTO mode, you can call
  ```
  rosservice call /controller_ns/test_joint_trajectory_controller/monitor_cartesian_speed "data: false"
  ```
