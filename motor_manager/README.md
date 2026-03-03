# motor_manager

Top-level library that ties everything together: loads YAML config, instantiates masters, drivers, and controllers, and runs the control loop.

## Behaviour

- **Constructor** — Takes a config file path; loads it and calls `initialize()` (so all masters and controllers are set up before you call `start()`).
- **start()** / **stop()** — Call `activate()` / `deactivate()` on all masters.
- **update(is_interrupted, status, command, size)** — Each cycle: receive on all masters; if not interrupted, enable if needed, then read status, check it, write command; transmit on all masters. If interrupted, disables. Returns `true` when all controllers are disabled.
- **period()**, **number_of_controllers()** — From config and slave count.

Config must define `masters` (with `type`, e.g. `ethercat`, and per-master `slaves`) and `drivers` (with `type`, e.g. `minas`, and `param_file`). Slave `controller_index` must be 0, 1, 2, … (sequential). The library creates one EthercatMaster per master entry, one MinasDriver per driver entry, and one EthercatController per slave; each controller is given the corresponding master and driver in `initialize()`.

## Dependencies

- **motor_interface** — Types and interfaces.
- **ethercat** — EthercatMaster, EthercatController.
- **minas** — MinasDriver.
- **yaml-cpp** — Config and driver param loading.

Link with `motor_manager::motor_manager`.
