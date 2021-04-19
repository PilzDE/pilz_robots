^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2021-04-19)
------------------
* Ports the driver to noetic. Includes moveing the trajectory planner to moveit
  * changes the references of the pilz_command_planner to the pilz_industrial_command_planner in moveit
  * fixes compatibility with ubuntu 20, noetic and colcon
  * changes CI to noetic and ubuntu 20
* Fix update routine in unittest of PilzJointTrajectoryController
* Contributors: Pilz GmbH and Co. KG

0.5.21 (2020-11-23)
-------------------

0.5.20 (2020-11-17)
-------------------
* Remove ROSNotOKException (#458)
* Fix wrong include for logger mock (#461)
* Contributors: Pilz GmbH and Co. KG

0.5.19 (2020-09-07)
-------------------
* Add tolerance to speed-limit acceptance-test (#436)
* Contributors: Pilz GmbH and Co. KG

0.5.18 (2020-07-02)
-------------------
* Make AsyncTest header-only
* Extends robot mock
* Fixes result of follow_joint_trajectory action
* Rename triggerCancellingOfActiveGoal() -> cancelActiveGoal()
* Contributors: Pilz GmbH and Co. KG

0.5.17 (2020-06-22)
-------------------
* Add cartesian speed monitoring to pilz joint trajectory controller
* Deactivate command interface in PilzJointTrajectoryController
* Add joint acceleration limits to pilz joint trajectory controller
* Contributors: Pilz GmbH and Co. KG

0.5.16 (2020-05-15)
-------------------
* Update and apply clang-format (#387)
* Contributors: Pilz GmbH and Co. KG

0.5.15 (2020-05-03)
-------------------
* Introduce goal_time_tolerance to PJTC function is_executing
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
* Minor fixes
* Contributors: Pilz GmbH and Co. KG

0.5.8 (2019-09-10)
------------------
* integrate clang-tidy via CMake flag
* Contributors: Pilz GmbH and Co. KG

0.5.7 (2019-08-29)
------------------
* Add is_executing service to joint trajectory controller
* Better handling consecutive calls on the hold service of joint trajectory controller
* Contributors: Pilz GmbH and Co. KG

0.5.6 (2019-06-12)
------------------

0.5.5 (2019-06-12)
------------------

0.5.4 (2019-05-27)
------------------

0.5.3 (2019-04-24)
------------------
* Fixes for new JointTrajectoryController Interface
* drop outdated can configuration
* Contributors: Pilz GmbH and Co. KG

0.5.2 (2019-02-21)
------------------
* Increase controller holding mode user feedback from INFO to WARN
* Contributors: Pilz GmbH and Co. KG

0.5.1 (2018-11-30)
------------------
* melodic release based on kinetic version 0.4.3
* Contributors: Pilz GmbH and Co. KG

0.5.0 (2018-11-07)
------------------
* fix joint trajectory controller due to new interface
* Contributors: Pilz GmbH and Co. KG

0.4.3 (2018-11-30)
------------------

0.4.2 (2018-11-08)
------------------

0.4.1 (2018-11-07)
------------------

0.4.0 (2018-11-06)
------------------
* joint trajectory controller with holding mode functionality
