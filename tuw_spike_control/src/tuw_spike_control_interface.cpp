#include <charconv>
#include <fstream>
#include <string>
#include <stdio.h>
#include <optional>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <boost/asio.hpp>
#include <iostream>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"
#include "tuw_spike_control/tuw_spike_control_interface.hpp"

#include "tuw_spike_control/interface_utils.hpp"

using namespace hardware_interface;

namespace tuw_spike_control {
static constexpr char TAG[] = "tuw_spike_control_interface";
static constexpr char LEFT_JOINT[] = "left_wheel_joint";
static constexpr char RIGHT_JOINT[] = "right_wheel_joint";

// Declare the io_context and serial port globally
boost::asio::io_context io_context;
boost::asio::serial_port serial(io_context);


TuwSpikeSystemInterface::TuwSpikeSystemInterface() = default;

TuwSpikeSystemInterface::~TuwSpikeSystemInterface()
{
  // If controller manager is shutdown via Ctrl + C, the on_deactivate methods won't be called.
  // A destructor with a call to on_deactivate is needed to ensure the device is stopped.
  on_deactivate(rclcpp_lifecycle::State());
}

CallbackReturn
TuwSpikeSystemInterface::on_init(const HardwareInfo &hardware_info) {
    if (SystemInterface::on_init(hardware_info) == CallbackReturn::ERROR) {
        return CallbackReturn::ERROR;
    }

    if (hardware_info.joints.size() != 2) {
        RCUTILS_LOG_ERROR_NAMED(
            TAG, "Not both joints specified");
        return CallbackReturn::ERROR;
    }

    try {

        reverse.push_back(read_param<bool>(hardware_info.joints[0], "reverse", false));
        reverse.push_back(read_param<bool>(hardware_info.joints[1], "reverse", false));

        command_motor_velocity.push_back(0.0);
        command_motor_velocity.push_back(0.0);

        state_motor_position.push_back(0.0);
        state_motor_position.push_back(0.0);

        state_motor_velocity.push_back(0.0);
        state_motor_velocity.push_back(0.0);

    } catch (std::runtime_error &e) {
        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing interface parameters: %s",
                                e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> TuwSpikeSystemInterface::export_state_interfaces() {
    std::vector<StateInterface> interfaces;
    interfaces.reserve(state_motor_position.size()*2);

    interfaces.emplace_back(LEFT_JOINT, HW_IF_POSITION,
                            &state_motor_position[0]);
    interfaces.emplace_back(LEFT_JOINT, HW_IF_VELOCITY,
                            &state_motor_velocity[0]);
    interfaces.emplace_back(RIGHT_JOINT, HW_IF_POSITION,
                            &state_motor_position[1]);
    interfaces.emplace_back(RIGHT_JOINT, HW_IF_VELOCITY,
                            &state_motor_velocity[1]);


    return interfaces;
}

std::vector<CommandInterface> TuwSpikeSystemInterface::export_command_interfaces() {
    std::vector<CommandInterface> interfaces;
    interfaces.emplace_back(LEFT_JOINT, HW_IF_VELOCITY,
                            &command_motor_velocity[0]);
    interfaces.emplace_back(RIGHT_JOINT, HW_IF_VELOCITY,
                            &command_motor_velocity[1]);
    return interfaces;
}

return_type TuwSpikeSystemInterface::read(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) {
    (void)time;
    (void)period;

    std::string p2_str = "P2C0";
    std::string p3_str = "P3C0";
    int p2_speed = INT32_MAX;
    int p2_apos = INT32_MAX;
    int p3_speed = INT32_MAX;
    int p3_apos = INT32_MAX;

    // Buffer to store incoming data
    std::vector<char> buffer(128);  // Adjust size as needed
    // Read data from serial port
    boost::system::error_code error;
    
    std::size_t bytes_read = 128;

    while(bytes_read == 128) {
        bytes_read = serial.read_some(boost::asio::buffer(buffer), error);

        std::string current;
        if (error) {
            std::cerr << "Error reading from serial port: " << error.message() << std::endl;
        } else {
            // std::cout << "Read " << bytes_read << " bytes\n";
            // process buffer
            for (char c : buffer) {
                if (c == '\n') {
                    // line complete: check for completeness
                    if (current.size() >= 5) {
                        // newer info possibly available
                        auto first_space = current.find(' ');
                        auto second_space = current.find(' ', first_space + 2);
                        auto third_space = current.find(' ', second_space + 2);

                        if (first_space != std::string::npos && second_space != std::string::npos && third_space != std::string::npos) {
                            // Extract substring between the first and second space
                            std::string substring = current.substr(0, 4);

                            std::string speed = current.substr(first_space + 1, second_space - first_space - 1);
                            std::string apos = current.substr(second_space + 1, third_space - second_space - 1);

                            if (speed.size() > 0 && apos.size() > 0) {
                                if (!substring.compare(p2_str)) {
                                    try {
                                        p2_speed = std::stoi(speed);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing speed_2: %s", e.what());
                                    }
                                    try {
                                        p2_apos = std::stoi(apos);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing pos_2: %s", e.what());
                                    }
                                } else if (!substring.compare(p3_str)) {
                                    try {
                                        p3_speed = std::stoi(speed);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing speed_3: %s", e.what());
                                    }
                                    try {
                                        p3_apos = std::stoi(apos);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing pos_3: %s", e.what());
                                    }
                                }
                            }
                        }
                    }
                    current.clear();
                } else {
                    current += c;
                }
            }
        }
    }
    if (reverse[0]) {
        p2_apos = -p2_apos;
        p2_speed = -p2_speed; 
    }
    if (reverse[1]) {
        p3_apos = -p3_apos;
        p3_speed = -p3_speed;
    }
    if (abs(p2_speed) != INT32_MAX) {
        // new values read from port 2
        state_motor_velocity[0] = 2.0 * M_PI * p2_speed / 33;
        state_motor_position[0] = 1.0 * p2_apos / 180.0 * M_PI; 
    }
    if (abs(p3_speed) != INT32_MAX) {
        // new values read from port 3
        state_motor_velocity[1] = 2.0 * M_PI * p3_speed / 33;
        state_motor_position[1] = 1.0 * p3_apos / 180.0 * M_PI;
    }

    return return_type::OK;
}

return_type TuwSpikeSystemInterface::write(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) {
    (void)time;
    (void)period;

    double velocity_left = command_motor_velocity[0];
    double velocity_right = command_motor_velocity[1];

    if (reverse[0]) {
        velocity_left = -velocity_left;
    }
    if (reverse[1]) {
        velocity_right = -velocity_right;
    }

    std::string message = "port 2; set " +  std::to_string(velocity_left / (2*M_PI)) + "; port 3; set " +  std::to_string(velocity_right / (2*M_PI)) + ";\r";
    boost::asio::write(serial, boost::asio::buffer(message)); 

    return return_type::OK;
}

CallbackReturn TuwSpikeSystemInterface::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    try {
        // Define serial port settings
        std::string port_name = "/dev/ttyS0";
        unsigned int baud_rate = 115200;

        // Open the serial port
        serial.open(port_name);

        // Set the baud rate
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  
        std::string cmd = "echo 0;\r";
        boost::asio::write(serial, boost::asio::buffer(cmd));
        cmd = "plimit 1; port 2; combi 0 1 0 2 0 3 0; select 0 ; selrate 10; pid_diff 2 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;\r";
        boost::asio::write(serial, boost::asio::buffer(cmd));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        cmd = "port 3; combi 0 1 0 2 0 3 0; select 0; selrate 10; pid_diff 3 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;\r";
        boost::asio::write(serial, boost::asio::buffer(cmd));
    
    } catch (std::runtime_error &e) {
        RCUTILS_LOG_ERROR_NAMED(TAG, "Error configuring hardware: %s",
                                e.what());
        return CallbackReturn::ERROR;
    }

    // TODO: load firmware if necessary
    return CallbackReturn::SUCCESS;
}

CallbackReturn TuwSpikeSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    // end select
    std::string end_message = "port 2; select; set 0; port 3; select; set 0;\r";
    boost::asio::write(serial, boost::asio::buffer(end_message)); 

    // Close serial port
    serial.close();
    return CallbackReturn::SUCCESS;
}

CallbackReturn TuwSpikeSystemInterface::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    // do the same steps as in cleanup
    return on_cleanup(previous_state);
}

CallbackReturn TuwSpikeSystemInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return CallbackReturn::SUCCESS;
}

CallbackReturn TuwSpikeSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    // do the same steps as in cleanup
    return on_cleanup(previous_state);
}

CallbackReturn TuwSpikeSystemInterface::on_error(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    // do the same steps as in cleanup
    return on_cleanup(previous_state);
}

} // namespace tuw_spike_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tuw_spike_control::TuwSpikeSystemInterface,
                       hardware_interface::SystemInterface)