^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_ikfast_manipulator_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.5.19 (2020-09-07)
-------------------
* Move ikfast plugin test to prbt_moveit_config (#450)
* Contributors: Pilz GmbH and Co. KG

0.5.18 (2020-07-02)
-------------------

0.5.17 (2020-06-22)
-------------------

0.5.16 (2020-05-15)
-------------------
* Update and apply clang-format (#387)
* Make test-subfolder-names consistent (#380)
* Contributors: Pilz GmbH and Co. KG

0.5.15 (2020-05-03)
-------------------

0.5.14 (2020-03-11)
-------------------

0.5.13 (2019-12-04)
-------------------

0.5.12 (2019-11-28)
-------------------

0.5.11 (2019-11-22)
-------------------
* update ikfast plugin (#204)
* drop unused lapack dependency (#186)
* Contributors: Pilz GmbH and Co. KG

0.5.10 (2019-10-08)
-------------------

0.5.9 (2019-10-07)
------------------

0.5.8 (2019-09-10)
------------------
* integrate clang-tidy via CMake flag
* Contributors: Pilz GmbH and Co. KG

0.5.7 (2019-08-29)
------------------

0.5.6 (2019-06-12)
------------------

0.5.5 (2019-06-12)
------------------
* enable aligned new in gcc7+

0.5.4 (2019-05-27)
------------------


0.5.3 (2019-04-24)
------------------
* Updated IK fast plugin
* Remove unused testfile
* drop outdated can configuration
* Contributors: Pilz GmbH and Co. KG

0.5.2 (2019-02-21)
------------------
* Fix sign-compare and deprecated warnings
* update IKfast plugin based on moveit version 0.10.8
* Contributors: Pilz GmbH and Co. KG

0.5.1 (2018-11-30)
------------------
* melodic release based on kinetic version 0.4.3
* Contributors: Pilz GmbH and Co. KG

0.5.0 (2018-11-07)
------------------

0.4.3 (2018-11-30)
------------------

0.4.2 (2018-11-08)
------------------

0.4.1 (2018-11-07)
------------------

0.4.0 (2018-11-06)
------------------
* add code coverage make target
* Contributors: Pilz GmbH and Co. KG

0.3.0 (2018-08-15)
------------------

0.2.1 (2018-07-19)
------------------
* Add <url> tag to all package.xml files

0.2.0 (2018-07-12)
------------------
* generate IKfast solver and plugin
* Contributors: Pilz GmbH and Co. KG

