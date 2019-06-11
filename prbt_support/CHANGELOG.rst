^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
