# Motor Manager

C++ library for managing motor masters, drivers, and controllers. Configured via YAML; supports EtherCAT (IgH) and Panasonic Minas drivers.

## Structure

| Directory | Description |
|-----------|-------------|
| `core/motor_interface` | Abstract interfaces: `MotorMaster`, `MotorDriver`, `MotorController` |
| `communications/ethercat` | EtherCAT master and controller (IgH) |
| `hardware/minas` | Panasonic Minas driver |
| `motor_manager` | Main library: loads config, owns masters/drivers/controllers, runs update loop |

## Build

```bash
cmake -B build
cmake --build build
```

**Dependencies:** CMake 3.16+, C++17, [yaml-cpp](https://github.com/jbeder/yaml-cpp), [IgH EtherCAT Master](https://etherlab.org/en/ethercat/).
