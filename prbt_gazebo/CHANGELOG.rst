^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.5.18 (2020-07-02)
-------------------

0.5.17 (2020-06-22)
-------------------

0.5.16 (2020-05-15)
-------------------
* Update and apply clang-format (#387)
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

0.5.4 (2019-05-27)
------------------

0.5.3 (2019-04-24)
------------------
* Remove rosparam block no longer needed
* Add world_name argument to gazebo launch file.
* Install launch and config folder of prbt_gazebo
* Contributors: Pilz GmbH and Co. KG

0.5.2 (2019-02-21)
------------------
* With this change the integrationtest loads the blank.world which
  has no models.
* Install launch and config files
* Contributors: Pilz GmbH and Co. KG

0.4.6 (2019-01-18)
------------------

0.4.5 (2019-01-16)
------------------

0.4.4 (2019-01-16)
------------------
* Provide prbt_gazebo package
* Contributors: Pilz GmbH and Co. KG
