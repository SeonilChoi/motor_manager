# ethercat

EtherCAT implementation using the [IgH EtherCAT Master](https://etherlab.org/en/ethercat/) (ecrt.h).

## Components

- **EthercatMaster** — One EtherCAT master. Construct from `master_config_t` (id, number_of_slaves, master_index). Creates domain; `initialize()`, `activate()` / `deactivate()`, `transmit()` / `receive()` drive the realtime exchange. Exposes `master()`, `domain()`, `domain_pd()` for PDO access.
- **EthercatController** — One slave/axis. Construct from `slave_config_t` (controller_index, master_id, driver_id, alias, position, vendor_id, product_id). In `initialize(master, driver)` it gets the slave config, registers SDOs from the driver’s items and PDOs from the driver’s interfaces, and fills `offset_[]` for PDO mapping. `enable()` / `disable()` use the driver’s state machine; `read()` / `write()` move data between `motor_frame_t` and the process data image.

## Dependencies

- **motor_interface** — Base interfaces and types.
- **IgH EtherCAT Master** — System library and ecrt.h.

Link with `ethercat::ethercat`.
