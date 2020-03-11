^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_hardware_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.14 (2020-03-11)
-------------------
* Trim firmware string.
* Rename waitForTopic() -> waitForMessage()
* Add Status Indicator that shows operation mode, speed override, HW status and ROS status
* Contributors: Pilz GmbH and Co. KG

0.5.13 (2019-12-04)
-------------------
* Use brake-test definitions from pilz_msgs
* Contributors: Pilz GmbH and Co. KG

0.5.12 (2019-11-28)
-------------------
* Remove srv definition for speed override (moved to the package pilz_msgs)
* Contributors: Pilz GmbH and Co. KG

0.5.11 (2019-11-22)
-------------------
* Fix clang compiler errors (#283)
* Contributors: Pilz GmbH and Co. KG

0.5.10 (2019-10-08)
-------------------
* Add missing dependency on tf2_geometry_msgs (`#264 <https://github.com/PilzDE/pilz_robots/issues/264>`_)
* Minor fixes
* Contributors: Pilz GmbH and Co. KG

0.5.9 (2019-10-07)
------------------
* Add service for getting the global speed override
* Add modbus register for enabling temporary movement
* Add Frame speed monitoring
* Test fixes and improvements
* Contributors: Pilz GmbH and Co. KG

* Add speed observing dependent on operation mode
* Contributors: Pilz GmbH and Co. KG

0.5.8 (2019-09-10)
------------------
* add missing transition to STO state machine
* revise STO specification
* integrate clang-tidy via CMake flag
* Contributors: Pilz GmbH and Co. KG

0.5.7 (2019-08-29)
------------------
* Add state machine for STO handling to allow skipping of hold/unhold if needed
* add operation mode functionality
* add write capability to PilzModbusReadClient, rename PilzModbusReadClient -> PilzModbusClient
* enter hold mode at braketest execution
* automatically determine range for reading modbus registers
* only read modbus registers that are explicitly configured (not in a single block)
* separate api definitions for read and write
* brake test result can be sent to FS controller
* Contributors: Pilz GmbH and Co. KG

0.5.6 (2019-06-12)
------------------
* Essentially reverts wrong fix (depend on canopen_chain_node) from 0.5.5

0.5.5 (2019-06-12)
------------------
* Add missing depend (CATKIN_DEPENDS and <run_depend>) on canopen_chain_node

0.5.4 (2019-05-27)
------------------
* increased modbus response timeout to 20ms
* publish brake test requests obtained from safety controller via modbus
* sto_modbus_adapter waits for the services to appear instead of throwing exceptions
* Add ability to execute a braketest on each drive.
* Add service to access the active operation mode
* Contributors: Pilz GmbH and Co. KG


0.5.3 (2019-04-24)
------------------
* cleanup CMakeLists of prbt_hardware_support
* update the documentation
* more precise error output when failing to read modbus register
* Contributors: Pilz GmbH and Co. KG

0.5.2 (2019-02-21)
------------------
* Update used pipeline in test from command_planner to pilz_command_planner
* Fix PilzModbusReadClient unittest
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
* Fix missing include on std_srvs

0.4.1 (2018-11-07)
------------------
* Use Modbus API v2 due to wrongly specified version 1

0.4.0 (2018-11-06)
------------------
* Modbus client node and STO modbus adapter node for Stop 1 functionality
