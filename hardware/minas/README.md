# minas

Driver implementation for **Panasonic Minas** servo drives (CIA 402 / EtherCAT). Loads SDO and PDO mapping from a YAML parameter file.

## Component

- **MinasDriver** — Extends `MotorDriver`. Constructor takes `driver_config_t` (id, pulse_per_revolution, rated_torque, unit_torque, limits, speeds, profile values). `loadParameters(param_file)` reads YAML: `items` (SDO index/subindex/type/value) and `interfaces` (PDO entries; IDs 0–3 RX, 4–8 TX). Converts position/velocity/torque using PPR and rated/unit torque. Implements `isEnabled` / `isDisabled` / `isReceived` for the standard state machine (statusword/controlword).

## Param file (YAML)

- **items** — List of `id`, `index`, `subindex`, `type` (u8/u16/u32/…), and `value` (or driver-filled from config for limits, profile, etc.).
- **interfaces** — List of PDO entries: `id` (0–8), `index`, `subindex`, `size`, `type`. First two entries are typically RXPDO/TXPDO indices (id 98/99).

## Dependencies

- **motor_interface** — Base driver interface and types.
- **yaml-cpp** — For loading the param file.

Link with `minas::minas`.
