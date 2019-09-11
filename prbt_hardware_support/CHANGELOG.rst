^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prbt_hardware_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.11 (2019-09-11)
-------------------
* add missing transition to STO state machine
* revise STO specification
* integrate clang-tidy via CMake flag
* Contributors: Pilz GmbH and Co. KG

0.4.10 (2019-09-03)
-------------------
* Add state machine for STO handling to allow skipping of hold/unhold if needed
* add operation mode functionality
* add write capability to PilzModbusReadClient, rename PilzModbusReadClient -> PilzModbusClient
* enter hold mode at braketest execution
* automatically determine range for reading modbus registers
* only read modbus registers that are explicitly configured (not in a single block)
* separate api definitions for read and write
* brake test result can be sent to FS controller
* Contributors: Pilz GmbH and Co. KG

0.4.9 (2019-06-19)
------------------
* increased modbus response timeout to 20ms
* publish brake test requests obtained from safety controller via modbus
* sto_modbus_adapter waits for the services to appear instead of throwing exceptions
* Add ability to execute a braketest on each drive.
* Contributors: Pilz GmbH and Co. KG


0.4.8 (2019-04-24)
------------------
* cleanup CMakeLists of prbt_hardware_support
* update the documentation
* Contributors: Pilz GmbH and Co. KG

0.4.7 (2019-02-15)
------------------
* drop outdated can configuration
* Contributors: Pilz GmbH and Co. KG

0.4.6 (2019-01-18)
------------------
* Update used pipeline in test from command_planner to pilz_command_planner

0.4.5 (2019-01-16)
------------------

0.4.4 (2019-01-16)
------------------
* Fix PilzModbusReadClient unittest

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
