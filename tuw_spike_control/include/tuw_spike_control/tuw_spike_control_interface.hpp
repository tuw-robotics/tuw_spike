#ifndef TUW_SPIKE_CONTROL__TUWSPIKE_SYSTEM_INTERFACE_HPP_
#define TUW_SPIKE_CONTROL__TUWSPIKE_SYSTEM_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"

namespace tuw_spike_control {

/// @brief ros2_control hardware interface for a single motor of TuwSpike
/// @details
/// The motor driver is a simple PWM EN/DIR Type H-Bridge driver.
/// Mounted to the axle is a Hall-Effect absolute magnetic encoder.
/// The hardware interface runs a velocity controller, to provide a velocity
/// command interface. The interface also provides position and velocity
/// feedback.
class TuwSpikeSystemInterface : public hardware_interface::SystemInterface {
  public:
    TuwSpikeSystemInterface();
    ~TuwSpikeSystemInterface() override;

  private:
    CallbackReturn
    on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_error(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;
    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type
    write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    
    std::vector<double> state_motor_position;
    std::vector<double> state_motor_velocity;
    std::vector<double> command_motor_velocity;

    std::vector<bool> reverse;

    uint8_t left_wheel_port, right_wheel_port;

    int left_wheel_offset = 0, right_wheel_offset = 0;

};

} // namespace tuw_spike_control

#endif // TUW_SPIKE_CONTROL__TUWSPIKE_SYSTEM_INTERFACE_HPP_
