# motor_interface

Header-only core: abstract interfaces and shared types used by all communication and driver packages.

## Interfaces

- **MotorMaster** — One bus master (e.g. one EtherCAT master). `initialize()`, `activate()` / `deactivate()`, `transmit()` / `receive()`.
- **MotorDriver** — Device-specific parameters and scaling (counts ↔ position/velocity/torque). Loads from YAML; exposes `items()` (SDO) and `interfaces()` (PDO), plus state-machine helpers `isEnabled` / `isDisabled` / `isReceived`.
- **MotorController** — One logical axis: ties a master and a driver, `initialize(master, driver)`, `enable()` / `disable()`, `read()` / `write()` / `check()` with `motor_frame_t`.

## Types

- **motor_frame_t** — controlword, statusword, errorcode, position, velocity, torque; plus `number_of_target_interfaces` and `target_interface_id[]` for write.
- **slave_config_t**, **master_config_t**, **driver_config_t** — config for controller, master, driver.
- **entry_table_t**, **DataType**, **DriverState** — PDO/SDO entries and state machine.

No implementation here; link against `motor_interface::motor_interface` to get include path and C++17.
