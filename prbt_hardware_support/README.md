# Overview
The prbt_hardware_support package contains files needed to control hardware functions of the PRBT manipulator like STO for Stop1 functionality. The Pilz hardware PNOZmulti and PSS4000 is supported using a Modbus connection.

There is no need to call these launch files directly; they are included from `prbt_support/robot.launch`.

# Safe stop 1 (SS1)
The STO function (“Safe torque off”) of the robot arm is a safety function to immediately turn off torque of the drives. To allow a
controlled stop, the safety controller is allowed to delay the STO signal by several milliseconds. This package opens a
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

# Architecture
![Component diagram](doc/architecture_overview.png)

# ROS API

## ModbusClient
A Modbus client (for usage with the PNOZmulti or PSS4000) can be started with `roslaunch prbt_hardware_support modbus_read_client.launch`.

### Published Topics
- ~/pilz_modbus_node/modbus_read (prbt_hardware_support/ModbusMsgInStamped)
  - Holds information about the modbus holding register. Timestamp is only updated if the register content changed.
- ~/prbt/brake_test_required (std_msgs/Bool)
  - True, if a brake test is required, false otherwise.

### Parameters
- modbus_server_ip
- modbus_server_port
- index_of_first_register_to_read
- num_registers_to_read
- modbus_connection_retries (default: 10)
- modbus_connection_retry_timeout - timeout between retries (default: 1s)
- modbus_response_timeout (default: 20ms)
- modbus_topic_name (default: "/pilz_modbus_node/modbus_read")

**Please note:**
- The parameters ``modbus_response_timeout`` and ``modbus_topic_name`` are
important for the Safe stop 1 functionality and must NOT be given, if the
``modbus_read_node`` is used as part of the Safe stop 1 functionality.
If the parameters are not given the default values for these parameters are used.

## StoModbusAdapterNode
The ``PilzStoModbusAdapterNode`` is noticed via the topic `/pilz_modbus_node/modbus_read` if the STO is true or false and reacts as follows calling the corresponding services of the controllers and drivers:
- **STO true:**
enable drives, unhold controllers
- **STO false:**
hold controllers, disable drives

## ModbusAdapterBrakeTestNode
The ``ModbusAdapterBrakeTestNode`` is noticed via the topic `/pilz_modbus_node/modbus_read` if the PSS4000 requests a brake test or if a brake test request is no longer prevailing. As a result, the requirement status is published on `/prbt/brake_test_required`.

## BraketestExecutorNode
The ``BraketestExecutorNode`` offers the `/execute_braketest` service which, in interaction with the ``CanOpenBraketestAdapter``,
executes a braketest on each drive of the manipulator.
