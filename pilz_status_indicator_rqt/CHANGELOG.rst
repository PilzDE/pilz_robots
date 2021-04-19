^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_status_indicator_rqt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2021-04-19)
------------------
* Ports the driver to noetic. Includes moveing the trajectory planner to moveit
  * changes the references of the pilz_command_planner to the pilz_industrial_command_planner in moveit
  * fixes compatibility with ubuntu 20, noetic and colcon
  * changes CI to noetic and ubuntu 20
* Contributors: Pilz GmbH and Co. KG

0.5.21 (2020-11-23)
-------------------
* Update references to OperationMode msg and GetOperationMode srv (moved to pilz_msgs)
* Contributors: Pilz GmbH and Co. KG

0.5.20 (2020-11-17)
-------------------
* Update package.xml for python 3
* Contributors: Pilz GmbH and Co. KG

0.5.19 (2020-09-07)
-------------------
* Hide currently unsupported ui elements
* Contributors: Pilz GmbH and Co. KG

0.5.18 (2020-07-02)
-------------------

0.5.17 (2020-06-22)
-------------------

0.5.16 (2020-05-15)
-------------------

0.5.15 (2020-05-03)
-------------------

0.5.14 (2020-03-11)
-------------------
* Adds basic view for operation_mode, status, speed
* Contributors: Pilz GmbH and Co. KG
