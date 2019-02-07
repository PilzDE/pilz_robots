<!--
Copyright (c) 2018 Pilz GmbH & Co. KG

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
-->

# Acceptance Test for the stop1 feature
These acceptance tests check that the real robot system reacts to STO messages accordingly.

## Prerequisites for each test
  - Properly connect and startup the robot and power cabinet containing the PNoz-Multi.
    Make sure an emergency stop is within reach.

## 1. Starting while acknowledge button is pressed
### Test Sequence
  1. Press the acknowledge button. Make sure the green light on the power cabinet is on.
     Run `roslaunch psir_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_command_planner`
  2. Release the acknowledge button. Try to move the robot.
  3. Press the acknowledge button. Make sure the green light on the power cabinet is on.
  4. Using Rviz perform a long motion during which you release the acknowledge button.
### Expected Results
  1. The robot starts properly and is moveable via Rviz.
  2. The robot cannot be moved.
  3. A "Click" indicates the reenabling of the drives. The robot can be moved.
  4. Upon the release of the button the robot stops smoothly.

## 2. Starting without acknowledge button beeing pressed
### Test Sequence
  1. Do not press the acknowledge button.
     Run `roslaunch psir_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_command_planner`
     After a about 10 seconds press the safety cabinet.
  2. Try to move the Robot via Rviz.

### Expected Results
  1. A "Click" indicates the enabling of the drives.
  2. The robot can be moved.

## 3. Loosing ethernet connection during motion
### Test Sequence
  1. Same as Test 1
  2. Move the robot. During the motion pull the ethernet cable from the power cabinet

### Expected Results
  1. Same as Test 1
  2. Robot should stop immediatly and smoothly.
