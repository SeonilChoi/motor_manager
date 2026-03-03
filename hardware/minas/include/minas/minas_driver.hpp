#ifndef MINAS_MINAS_DRIVER_HPP_
#define MINAS_MINAS_DRIVER_HPP_

#include "motor_interface/motor_driver.hpp"

namespace minas {

inline constexpr uint16_t SW_FAULT                = 0x0008;
inline constexpr uint16_t SW_READY_TO_SWITCH_ON   = 0x0021;
inline constexpr uint16_t SW_SWITCHED_ON          = 0x0023;
inline constexpr uint16_t SW_OPERATION_ENABLED    = 0x0027;
inline constexpr uint16_t SW_SWITCH_ON_DISABLED   = 0x0040;
inline constexpr uint16_t SW_SETPOINT_ACKNOWLEDGE = 0x1000;

inline constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;
inline constexpr uint16_t CW_SHUTDOWN          = 0x0006;
inline constexpr uint16_t CW_SWITCH_ON         = 0x0007;
inline constexpr uint16_t CW_DISABLE_OPERATION = 0x0007;
inline constexpr uint16_t CW_ENABLE_OPERATION  = 0x000F;
inline constexpr uint16_t CW_NEW_SETPOINT      = 0x003F;
inline constexpr uint16_t CW_FAULT_RESET       = 0x0080;

inline constexpr uint8_t ID_MAX_TORQUE           = 50;
inline constexpr uint8_t ID_MIN_POSITION_LIMIT   = 51;
inline constexpr uint8_t ID_MAX_POSITION_LIMIT   = 52;
inline constexpr uint8_t ID_MAX_MOTOR_SPEED      = 53;
inline constexpr uint8_t ID_PROFILE_VELOCITY     = 54;
inline constexpr uint8_t ID_PROFILE_ACCELERATION = 55;
inline constexpr uint8_t ID_PROFILE_DECELERATION = 56;
inline constexpr uint8_t ID_MAX_ACCELERATION     = 57;
inline constexpr uint8_t ID_MAX_DECELERATION     = 58;
inline constexpr uint8_t ID_RXPDO                = 98;
inline constexpr uint8_t ID_TXPDO                = 99;

class MinasDriver : public motor_interface::MotorDriver {
public:
    explicit MinasDriver(const motor_interface::driver_config_t& config)
    : motor_interface::MotorDriver(config) {}

    virtual ~MinasDriver() = default;

    void loadParameters(const std::string& param_file) override;

    bool isEnabled(const uint8_t* data, motor_interface::DriverState& driver_state, uint8_t* out) override;

    bool isDisabled(const uint8_t* data, motor_interface::DriverState& driver_state, uint8_t* out) override;

    bool isReceived(const uint8_t* data, uint8_t* out) override;

    double position(const int32_t value) override;

    double velocity(const int32_t value) override;

    double torque(const int16_t value) override;

    int32_t position(const double value) override;

    int32_t velocity(const double value) override;

    int16_t torque(const double value) override;

private:
    bool isReadyToSwitchOn(uint16_t sw) { return (sw & 0x006F) == SW_READY_TO_SWITCH_ON; }

    bool isSwitchedOn(uint16_t sw) { return (sw & 0x006F) == SW_SWITCHED_ON; }

    bool isOperationEnabled(uint16_t sw) { return (sw & 0x006F) == SW_OPERATION_ENABLED; }

    bool isSwitchOnDisabled(uint16_t sw) { return (sw & 0x006F) == SW_SWITCH_ON_DISABLED; }

    bool isFault(uint16_t sw) { return (sw & SW_FAULT); }

    bool isSetpointAcknowledge(uint16_t sw) { return (sw & 0x1000) == SW_SETPOINT_ACKNOWLEDGE; }
};

} // namespace minas
#endif // MINAS_MINAS_DRIVER_HPP_