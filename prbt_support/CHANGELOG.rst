^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2021-04-19)
------------------
* Ports the driver to noetic. Includes moveing the trajectory planner to moveit
  * changes the references of the pilz_command_planner to the pilz_industrial_command_planner in moveit
  * fixes compatibility with ubuntu 20, noetic and colcon
  * changes CI to noetic and ubuntu 20
* Contributors: Pilz GmbH and Co. KG

0.5.21 (2020-11-23)
-------------------

0.5.20 (2020-11-17)
-------------------
* Fix acceptancetest_joint_limits (#448)
* Contributors: Pilz GmbH and Co. KG

0.5.19 (2020-09-07)
-------------------
* Reduce planning context to urdf model in urdf_tests.test (#451)
* Increase acceleration limits. (#442)
* Contributors: Pilz GmbH and Co. KG

0.5.18 (2020-07-02)
-------------------
* Make AsyncTest header-only
* Contributors: Pilz GmbH and Co. KG

0.5.17 (2020-06-22)
-------------------
* Redirect start of fake_speed_override_node to separate launch file
* Contributors: Pilz GmbH and Co. KG

0.5.16 (2020-05-15)
-------------------
* Fix include_directories for prbt_support tests (#390)
* Update and apply clang-format (#387)
* Make test-subfolder-names consistent (#380)
* Fix/joint limits acceptance test (#381)
* Contributors: Pilz GmbH and Co. KG

0.5.15 (2020-05-03)
-------------------
* Add support for starting the robot without modbus
* Rename argument enable_safety_interface to iso10218_support.
* Moved system_info_node here from prbt_hardware_support
* Adopted default configuration for launchfiles
* Add README
* Enable starting ROS without connecting to safety controller
* Contributors: Pilz GmbH and Co. KG

0.5.14 (2020-03-11)
-------------------

0.5.13 (2019-12-04)
-------------------

0.5.12 (2019-11-28)
-------------------

0.5.11 (2019-11-22)
-------------------
* Fix clang compiler errors (#283)
* Contributors: Pilz GmbH and Co. KG

0.5.10 (2019-10-08)
-------------------

0.5.9 (2019-10-07)
------------------
* Add frame speed monitoring
* Contributors: Pilz GmbH and Co. KG

0.5.8 (2019-09-10)
------------------
* integrate clang-tidy via CMake flag
* Contributors: Pilz GmbH and Co. KG

0.5.7 (2019-08-29)
------------------
* Add default modbus server ip for pss4000
* Contributors: Pilz GmbH and Co. KG

0.5.6 (2019-06-12)
------------------

0.5.5 (2019-06-12)
------------------

0.5.4 (2019-05-27)
------------------
* instantiate pg70 xacro macro (due to change in prbt_grippers)
* Contributors: Pilz GmbH and Co. KG

0.5.3 (2019-04-24)
------------------
* Add acceptance test for joint position limits
* Relax joint limits.
* Replace the radian values for the position limits (they have been rounded too roughly).
* Added support for force-/torque sensors in gazebo
* allow gripper_name as outside property instead of passing it explicitly
* Add gripper brackets definition to prbt.xacro
* Remove unnecessary file test_context.launch
  This file is moved to pilz_trajectory_generation, where it is mainly used.
  The test urdf_tests can use the original file planning_context.launch.
* Add missing dependency on joint_state_controller
* Relax joint limits (recompute radian values and round up 5th decimal)
* Contributors: Pilz GmbH and Co. KG

0.5.2 (2019-02-21)
------------------
* drop outdated can configuration
* Fixup of mesh files due to errors in gazebo visualization
* make robot.launch file configurable with args
* Contributors: Pilz GmbH and Co. KG

0.5.1 (2018-11-30)
------------------
* melodic release based on kinetic version 0.4.3
* Contributors: Pilz GmbH and Co. KG

0.5.0 (2018-11-07)
------------------

0.4.3 (2018-11-30)
------------------
* Update readme

0.4.2 (2018-11-08)
------------------

0.4.1 (2018-11-07)
------------------

0.4.0 (2018-11-06)
------------------
* Enable prbt model to be used in combination with Gazebo
* Add improved values for mass and inertia for all links
* Contributors: Pilz GmbH and Co. KG

0.3.0 (2018-08-15)
------------------
* remove dependency on gripper

0.2.2 (2018-07-26)
------------------
* Move constants inside prbt_macro.xacro into local namespace.

0.2.1 (2018-07-19)
------------------
* Add <url> tag to all package.xml files
* test launch files and add missing dependencies

0.2.0 (2018-07-12)
------------------
* initial robot model for prbt with and without pg+70 gripper
* Contributors: Pilz GmbH and Co. KG
