# Motor Manager

C++ library for managing motor control over fieldbus. Configuration is YAML-based; currently supports **EtherCAT** (IgH) and **Panasonic Minas** servo drivers.

## Structure

- **core/motor_interface** — Abstract interfaces (`MotorMaster`, `MotorDriver`, `MotorController`) and shared types. Header-only.
- **communications/ethercat** — EtherCAT implementation: master and controller using IgH EtherCAT Master.
- **hardware/minas** — Panasonic Minas driver: SDO/PDO setup from YAML param files.
- **motor_manager** — Top-level orchestration: loads config, creates masters/drivers/controllers, runs the update loop.

## Build

```bash
cmake -B build
cmake --build build
```

**Requirements:** CMake 3.16+, C++17, [yaml-cpp](https://github.com/jbeder/yaml-cpp), [IgH EtherCAT Master](https://etherlab.org/en/ethercat/).

`update()` receives from slaves, runs enable/disable state machine, reads status, checks it, writes command, then transmits. Returns `true` when all controllers are disabled.
