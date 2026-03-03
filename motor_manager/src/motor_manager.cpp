#include <yaml-cpp/yaml.h>

#include "motor_manager/motor_manager.hpp"
#include "ethercat/ethercat_master.hpp"
#include "ethercat/ethercat_controller.hpp"
#include "minas/minas_driver.hpp"

motor_manager::MotorManager::MotorManager(const std::string& config_file)
{
    loadConfigurations(config_file);
    initialize();
}

void motor_manager::MotorManager::loadConfigurations(const std::string& config_file)
{
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) throw std::runtime_error("Failed to load configuration file.");

    period_ = root["period"].as<uint32_t>();

    YAML::Node masters = root["masters"];
    if (!masters || !masters.IsSequence()) throw std::runtime_error("Invalid masters configuration.");

    uint8_t s_idx{0};
    masters_.reserve(MAX_MASTER_SIZE);
    for (const auto& m : masters) {
        motor_interface::master_config_t m_cfg{};
        m_cfg.id = m["id"].as<uint8_t>();
        m_cfg.number_of_slaves = m["number_of_slaves"].as<uint8_t>();

        YAML::Node slaves = m["slaves"];
        if (!slaves || !slaves.IsSequence()) throw std::runtime_error("Invalid slaves configuration.");

        switch (toCommunicationType(m["type"].as<std::string>())) {
        case CommunicationType::Ethercat: {
            m_cfg.master_index = m["master_index"].as<unsigned int>();
            masters_[m_cfg.id] = std::make_unique<ethercat::EthercatMaster>(m_cfg);

            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                motor_interface::slave_config_t s_cfg{};
                s_cfg.controller_index = slaves[i]["controller_index"].as<uint8_t>();
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.alias = slaves[i]["alias"].as<uint16_t>();
                s_cfg.position = slaves[i]["position"].as<uint16_t>();
                s_cfg.vendor_id = slaves[i]["vendor_id"].as<uint32_t>();
                s_cfg.product_id = slaves[i]["product_id"].as<uint32_t>();

                controllers_[s_cfg.controller_index] = std::make_unique<ethercat::EthercatController>(s_cfg);
                s_idx++;
            }
            break;
        } default: {
            throw std::runtime_error("Invalid communication type.");
        }
        }
    }
    number_of_controllers_ = s_idx;

    YAML::Node drivers = root["drivers"];
    if (!drivers || !drivers.IsSequence()) throw std::runtime_error("Invalid drivers configuration.");

    drivers_.reserve(MAX_DRIVER_SIZE);
    for (const auto& d : drivers) {
        motor_interface::driver_config_t d_cfg{};
        d_cfg.id = d["id"].as<uint8_t>();
        d_cfg.pulse_per_revolution = d["pulse_per_revolution"].as<uint32_t>();
        d_cfg.rated_torque = d["rated_torque"].as<double>();
        d_cfg.unit_torque = d["unit_torque"].as<double>();
        d_cfg.lower = d["lower"].as<double>();
        d_cfg.upper = d["upper"].as<double>();
        d_cfg.speed = d["speed"].as<double>();
        d_cfg.acceleration = d["acceleration"].as<double>();
        d_cfg.deceleration = d["deceleration"].as<double>();
        d_cfg.profile_velocity = d["profile_velocity"].as<double>();
        d_cfg.profile_acceleration = d["profile_acceleration"].as<double>();
        d_cfg.profile_deceleration = d["profile_deceleration"].as<double>();

        switch (toDriverType(d["type"].as<std::string>())) {
        case DriverType::Minas: {
            drivers_[d_cfg.id] = std::make_unique<minas::MinasDriver>(d_cfg);
            break;
        } default: {
            throw std::runtime_error("Invalid driver type.");
        }
        }
        drivers_.at(d_cfg.id)->loadParameters(d["param_file"].as<std::string>());
    }
}

void motor_manager::MotorManager::initialize()
{
    frequency_ = NSEC_PER_SEC / period_;

    for (auto& m_iter : masters_) m_iter.second->initialize();

    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!controllers_[i]) {
            throw std::runtime_error("Controller index must be 0-based and sequential in config.");
        }
        uint8_t m_id = controllers_[i]->master_id();
        uint8_t d_id = controllers_[i]->driver_id();
        controllers_[i]->initialize(*masters_.at(m_id), *drivers_.at(d_id));
    }
}

void motor_manager::MotorManager::enable()
{
    uint8_t result[MAX_CONTROLLER_SIZE]{0};
    uint8_t sum{0};

    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!controllers_[i]) {
            throw std::runtime_error("Controller index must be 0-based and sequential in config.");
        }
        if (!result[i]) result[i] = controllers_[i]->enable();
        sum += result[i];
    }
    is_enable_ = sum == number_of_controllers_;
}

void motor_manager::MotorManager::disable()
{
    uint8_t result[MAX_CONTROLLER_SIZE]{0};
    uint8_t sum{0};
    
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!controllers_[i]) {
            throw std::runtime_error("Controller index must be 0-based and sequential in config.");
        }
        if (!result[i]) result[i] = controllers_[i]->disable();
        sum += result[i];
    }
    is_disabled_ = sum == number_of_controllers_;
}

void motor_manager::MotorManager::check(const motor_interface::motor_frame_t* status)
{
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!controllers_[i]) {
            throw std::runtime_error("Controller index must be 0-based and sequential in config.");
        }
        controllers_[i]->check(status[i]);
    }
}

void motor_manager::MotorManager::write(const motor_interface::motor_frame_t* command, const uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t idx = command[i].controller_index;
        if (idx >= MAX_CONTROLLER_SIZE || !controllers_[idx]) {
            throw std::runtime_error("Invalid controller index in write.");
        }
        controllers_[idx]->write(command[i]);
    }
}

void motor_manager::MotorManager::read(motor_interface::motor_frame_t* status)
{
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!controllers_[i]) {
            throw std::runtime_error("Controller index must be 0-based and sequential in config.");
        }
        controllers_[i]->read(status[i]);
    }
}

bool motor_manager::MotorManager::update(bool is_interrupted, motor_interface::motor_frame_t* status, motor_interface::motor_frame_t* command, const uint8_t size)
{
    for (auto& m_iter : masters_) m_iter.second->receive();

    if (is_interrupted) {
        disable();
    } else {
        if (!is_enable_) {
            enable();
        } else {
            read(status);
            check(status);
            write(command, size);
        }
    }

    for (auto& m_iter : masters_) m_iter.second->transmit();

    return is_disabled_;
}