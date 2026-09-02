#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tuw_spike_control/tuw_spike_control_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "Hello, world from tuw_spike_control demo!" << std::endl;

  // Instantiate the hardware interface provided by the
  // tuw_spike_control_interface library.
  tuw_spike_control::TuwSpikeSystemInterface interface;

  // The lifecycle callbacks are private overrides, so they are invoked through
  // the public hardware_interface::SystemInterface API.
  hardware_interface::SystemInterface & lifecycle = interface;

  // Minimal HardwareInfo with the two wheel joints the interface expects.
  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info.name = "tuw_spike";
  init_params.hardware_info.type = "system";
  init_params.hardware_info.rw_rate = 100;

  hardware_interface::ComponentInfo left_wheel;
  left_wheel.name = "left_wheel_joint";
  left_wheel.type = "joint";
  hardware_interface::ComponentInfo right_wheel;
  right_wheel.name = "right_wheel_joint";
  right_wheel.type = "joint";
  init_params.hardware_info.joints = {left_wheel, right_wheel};

  if (lifecycle.on_init(init_params) != hardware_interface::CallbackReturn::SUCCESS) {
    std::cerr << "on_init() failed" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "on_init() returned: SUCCESS" << std::endl;

  const rclcpp_lifecycle::State previous_state;
  const auto result = lifecycle.on_configure(previous_state);

  std::cout << "on_configure() returned: "
            << (result == hardware_interface::CallbackReturn::SUCCESS ? "SUCCESS"
                : result == hardware_interface::CallbackReturn::FAILURE ? "FAILURE"
                                                                       : "ERROR")
            << std::endl;

  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    rclcpp::shutdown();
    return 1;
  }

  if (lifecycle.on_activate(previous_state) != hardware_interface::CallbackReturn::SUCCESS) {
    std::cerr << "on_activate() failed" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "on_activate() returned: SUCCESS" << std::endl;

  // Grab the exported command interfaces and pick the two wheel velocity ones.
  // (export_command_interfaces() is the legacy hook this hardware still implements.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto command_interfaces = lifecycle.export_command_interfaces();
#pragma GCC diagnostic pop
  hardware_interface::CommandInterface * left_wheel_velocity = nullptr;
  hardware_interface::CommandInterface * right_wheel_velocity = nullptr;
  for (auto & command_interface : command_interfaces) {
    if (command_interface.get_interface_name() != hardware_interface::HW_IF_VELOCITY) {
      continue;
    }
    if (command_interface.get_prefix_name() == "left_wheel_joint") {
      left_wheel_velocity = &command_interface;
    } else if (command_interface.get_prefix_name() == "right_wheel_joint") {
      right_wheel_velocity = &command_interface;
    }
  }
  if (left_wheel_velocity == nullptr || right_wheel_velocity == nullptr) {
    std::cerr << "wheel velocity command interfaces not found" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Spin both wheels at 2 rad/s for ~3 seconds, then stop them.
  // set_value() only updates the command buffer; write() pushes it to the motors.
  constexpr double target_velocity = 2.0;   // rad/s
  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.02);

  int exit_code = 0;
  try {
    std::cout << "spinning left_wheel_joint and right_wheel_joint at "
              << target_velocity << " rad/s" << std::endl;
    for (int i = 0; i < 150; ++i) {
      (void)left_wheel_velocity->set_value(target_velocity);
      (void)right_wheel_velocity->set_value(target_velocity);
      lifecycle.write(time, period);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "stopping both wheels" << std::endl;
    (void)left_wheel_velocity->set_value(0.0);
    (void)right_wheel_velocity->set_value(0.0);
    lifecycle.write(time, period);
  } catch (const std::exception & e) {
    std::cerr << "error while driving the wheels: " << e.what() << std::endl;
    exit_code = 1;
  }

  // The TuwSpikeSystemInterface destructor runs on_deactivate()/on_cleanup(),
  // which stops both motors and closes the serial port.
  rclcpp::shutdown();
  return exit_code;
}
