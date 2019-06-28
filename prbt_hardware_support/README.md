# Overview
The prbt_hardware_support package contains files supporting the certification of a robot system including the PRBT manipulator according to DIN EN ISO 10218-1. As safety controllers the Pilz hardware PNOZmulti and PSS4000 are supported. A Modbus connection is used for the communication between ROS <-> safety controller.

There is no need to call these launch files directly; they are included from `prbt_support/robot.launch`.

## STO
The STO function (“Safe torque off”) of the robot arm is a safety function to immediately turn off torque of the drives.

## SBC
The SBC function ("Safe brake control") of the robot arm is a safety function which is used in conjunction with the STO and prevents a motion when the torque of the drives is turned off.

# Safe stop 1 (SS1)
To allow a controlled stop, the safety controller delays the STO signal by several milliseconds. This package opens a
modbus connection to the safety controller (PNOZmulti or PSS4000). The safety controller sends an emergency
stop signal via Modbus immediately so that ros_control has a short time interval to stop the drives via a brake ramp.
The TCP could for example brake on the current trajectory. After execution of the brake ramp, the drivers are halted. Even if ROS would fail, the safety controller turns off the motors via STO (that would be a Stop 0 then).

## Possible error cases and their handling

| Error cases                                             | Handling                                                |
| ------------------------------------------------------- | ------------------------------------------------------- |
| Modbus client crashes                                   | ROS system is shutdown which leads to an abrupt stop of the robot. |
| STO Modbus adapter crashes                              | ROS system is shutdown which leads to an abrupt stop of the robot. |
| Connection loss between PNOZmulti/PSS4000 & Modbus client        | Stop 1 is triggered                                     |
| System overload (messages don't arrive in time)         | In case a Stop 1 message does not arrive in time, the safety controller will automatically perform a hard stop. In case a Stop 1-release message does not get through, brakes will remain closed. |
| STO Modbus adapter cannot connect to stop services    | ROS system will not start.                              |
| STO Modbus adapter cannot connect to recover services | Node does start and robot can be moved until a stop is triggered. Afterwards the brakes will remain closed. |

# Brake tests
Brake tests are an integral part of the SBC, since they detect misfunctions of the brakes or the brake control in general. Brake tests for each drive have to be executed at a regular interval. When the safety controller requests brake tests, they have to be executed within 1 hour, else the robot cannot be moved anymore.

# Operation Modes
The robot system can be controlled in various modes.

These modes are:
  - T1: Speed reduced to 250 mm/s (each robot-frame), enabling switch must be pressed
  - T2: The robot moves at full speed but still the enabling switch must be pressed
  - AUTOMATIC: The robot moves at full speed and follows a predefined program/process. No enabling switch is needed but safety has to be ensured by safety peripherie (fences, light curtains, ...).

See DIN EN ISO 10218-1 for more details or contact us: ros@pilz.de

# Architecture
![Component diagram](doc/architecture_overview.png)

# ROS API

## ModbusClient
A Modbus client (for usage with the PNOZmulti or PSS4000) can be started with `roslaunch prbt_hardware_support modbus_client.launch`.

### Published Topics
- ~/pilz_modbus_client_node/modbus_read (prbt_hardware_support/ModbusMsgInStamped)
  - Holds information about the modbus holding register. Timestamp is only updated if the register content changed.

### Parameters
- modbus_server_ip
- modbus_server_port
- index_of_first_register_to_read
- num_registers_to_read
- modbus_connection_retries (default: 10)
- modbus_connection_retry_timeout - timeout between retries (default: 1s)
- modbus_response_timeout (default: 20ms)
- modbus_read_topic_name (default: "/pilz_modbus_client_node/modbus_read")
- modbus_write_service_name (default: "/pilz_modbus_client_node/modbus_write")

**Please note:**
- The parameters ``modbus_response_timeout`` and ``modbus_read_topic_name`` are
important for the Safe stop 1 functionality and must NOT be given, if the
``pilz_modbus_client_node`` is used as part of the Safe stop 1 functionality.
If the parameters are not given the default values for these parameters are used.

## ModbusAdapterStoNode
The ``ModbusAdapterSto`` is noticed via the topic `/pilz_modbus_client_node/modbus_read` if the STO is true or false and reacts as follows calling the corresponding services of the controllers and drivers:
- **STO true:**
enable drives, unhold controllers
- **STO false:**
hold controllers, disable drives

## ModbusAdapterBrakeTestNode
The ``ModbusAdapterBrakeTestNode`` offers the `/prbt/brake_test_required` service which informs if the PSS4000 requests
a brake test or if a brake test request is no longer prevailing.

## BraketestExecutorNode
The ``BraketestExecutorNode`` offers the `/execute_braketest` service which, in interaction with the ``CanOpenBraketestAdapter``,
executes a braketest on each drive of the manipulator. This can only be done, if the robot is stopped. So, if you want to execute a braketest, ensure that the robot stands still.

## ModbusAdapterOperationModeNode
The ``ModbusAdapterOperationModeNode`` offers the `/get_operation_mode` service for accessing the active operation mode.

Use `rosmsg show prbt_hardware_support/OperationModes` to see the definition of each value.
