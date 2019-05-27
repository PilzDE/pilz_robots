^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_hardware_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
