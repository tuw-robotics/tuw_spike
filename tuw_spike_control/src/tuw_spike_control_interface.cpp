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
#include <fstream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_path.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"
#include "tuw_spike_control/tuw_spike_control_interface.hpp"

#include "tuw_spike_control/interface_utils.hpp"

using namespace hardware_interface;

namespace tuw_spike_control {
//static constexpr char SERIAL_PORT[]  = "/dev/ttyAMA0";
static constexpr char SERIAL_PORT[]  = "/dev/ttyUSB0";
static constexpr char FIRMWARE_FILENAME[]  = "2025-01-22_firmware.bin";
static constexpr char SIGNATURE_FILENAME[] = "2025-01-22_signature.bin";
static constexpr char TAG[] = "tuw_spike_control_interface";
static constexpr char LEFT_JOINT[] = "left_wheel_joint";
static constexpr char RIGHT_JOINT[] = "right_wheel_joint";

// Declare the io_context and serial port globally
boost::asio::io_context io_context;
boost::asio::serial_port serial(io_context);
boost::asio::streambuf buf;

// read a binary file and return the content as a std::vector<unsigned char>
std::vector<unsigned char> readBinaryFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(file), {});
    return buffer;
}

// read a line from the serial port
std::string serialReadLine() {
    boost::asio::read_until(serial, buf, '\n');
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);
    return line;
}

// read from the serial port until "BHBL>" is read
void getPrompt() {
    std::string bootloader_str = "BHBL>";
    while (true) {
        std::string line = serialReadLine();
        if (line.find(bootloader_str) != std::string::npos) {
            break;
        }
    }
}

// calculate the checksum for the firmware
uint32_t checksum(const std::vector<uint8_t>& data) {
    uint32_t u = 1;
    for (size_t i = 0; i < data.size(); ++i) {
        if ((u & 0x80000000) != 0) {
            u = (u << 1) ^ 0x1d872b41;
        } else {
            u = u << 1;
        }
        u = (u ^ data[i]) & 0xFFFFFFFF;
    }
    return u;
}

void loadFirmware() {
    std::filesystem::path firmware_directory = ament_index_cpp::get_package_share_path("tuw_spike_control") / "firmware";
    std::string firmwarePath  = (firmware_directory / FIRMWARE_FILENAME).string();
    std::string signaturePath = (firmware_directory / SIGNATURE_FILENAME).string();

    RCUTILS_LOG_INFO_NAMED(TAG, "load Firmware: %s", firmwarePath.c_str());

    std::vector<unsigned char> firmware = readBinaryFile(firmwarePath);
    std::vector<unsigned char> signature = readBinaryFile(signaturePath);
   
    // clear current image
    boost::asio::write(serial, boost::asio::buffer("clear\r", 6));
    getPrompt();

    // write firmware to serial port
    std::string loadCommand = "load " + std::to_string(std::filesystem::file_size(firmwarePath)) + " " + std::to_string(checksum(firmware)) + "\r";
    boost::asio::write(serial, boost::asio::buffer(loadCommand));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    boost::asio::write(serial, boost::asio::buffer("\x02", 1));
    boost::asio::write(serial, boost::asio::buffer(firmware));
    boost::asio::write(serial, boost::asio::buffer("\x03\r", 2));
    getPrompt();

    // write signature to serial port
    std::string signatureCommand = "signature " + std::to_string(std::filesystem::file_size(signaturePath)) + "\r";   
    boost::asio::write(serial, boost::asio::buffer(signatureCommand));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    boost::asio::write(serial, boost::asio::buffer("\x02", 1));
    boost::asio::write(serial, boost::asio::buffer(signature));
    boost::asio::write(serial, boost::asio::buffer("\x03\r", 2));
    getPrompt();

    boost::asio::write(serial, boost::asio::buffer("reboot\r", 7));

    // wait for 10 seconds until reboot is finished
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

}


TuwSpikeSystemInterface::TuwSpikeSystemInterface() = default;

TuwSpikeSystemInterface::~TuwSpikeSystemInterface()
{
  // If controller manager is shutdown via Ctrl + C, the on_deactivate methods won't be called.
  // A destructor with a call to on_deactivate is needed to ensure the device is stopped.
  on_deactivate(rclcpp_lifecycle::State());
}

CallbackReturn
TuwSpikeSystemInterface::on_init(const HardwareComponentInterfaceParams &params) {
    if (SystemInterface::on_init(params) == CallbackReturn::ERROR) {
        return CallbackReturn::ERROR;
    }

    const HardwareInfo &hardware_info = params.hardware_info;

    if (hardware_info.joints.size() != 2) {
        RCUTILS_LOG_ERROR_NAMED(
            TAG, "Not both joints specified");
        return CallbackReturn::ERROR;
    }

    try {

        reverse.push_back(read_param<bool>(hardware_info.joints[0], "reverse", false));
        reverse.push_back(read_param<bool>(hardware_info.joints[1], "reverse", false));
        left_wheel_port = read_param<uint8_t>(hardware_info.joints[0], "Port", 1);
        right_wheel_port = read_param<uint8_t>(hardware_info.joints[1], "Port", 0);


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

    std::string left_port_str = "P" + std::to_string(left_wheel_port) + "C0";
    std::string right_port_str = "P" + std::to_string(right_wheel_port) + "C0";
    int left_wheel_speed = INT32_MAX;
    int left_wheel_apos = INT32_MAX;
    int right_wheel_speed = INT32_MAX;
    int right_wheel_apos = INT32_MAX;

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
                                if (!substring.compare(left_port_str)) {
                                    try {
                                        left_wheel_speed = std::stoi(speed);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing speed_left: %s", e.what());
                                    }
                                    try {
                                        left_wheel_apos = std::stoi(apos);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing pos_left: %s", e.what());
                                    }
                                } else if (!substring.compare(right_port_str)) {
                                    try {
                                        right_wheel_speed = std::stoi(speed);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing speed_right: %s", e.what());
                                    }
                                    try {
                                        right_wheel_apos = std::stoi(apos);
                                    } catch (std::invalid_argument const& e) {
                                        RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing pos_right: %s", e.what());
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
        left_wheel_apos = -left_wheel_apos;
        left_wheel_speed = -left_wheel_speed; 
    }
    if (reverse[1]) {
        right_wheel_apos = -right_wheel_apos;
        right_wheel_speed = -right_wheel_speed;
    }
    if (abs(left_wheel_speed) != INT32_MAX) {
        // new values read from left wheel
        state_motor_velocity[0] = 2.0 * M_PI * left_wheel_speed / 33;
        state_motor_position[0] = 1.0 * (left_wheel_apos-(left_wheel_offset)) / 180.0 * M_PI; 
    }
    if (abs(right_wheel_speed) != INT32_MAX) {
        // new values read from right wheel
        state_motor_velocity[1] = 2.0 * M_PI * right_wheel_speed / 33;
        state_motor_position[1] = 1.0 * (right_wheel_apos-(right_wheel_offset)) / 180.0 * M_PI;
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

    // std::string message = "port " + std::to_string(left_wheel_port) + "; set " +  std::to_string(velocity_left / 18.84) + "; port " + std::to_string(right_wheel_port) + "; set " +  std::to_string(velocity_right / (18.84)) + ";\r";   // pwm
    std::string message = "port " + std::to_string(left_wheel_port) + "; set " +  std::to_string(velocity_left / (2*M_PI)) + "; port " + std::to_string(right_wheel_port) + "; set " +  std::to_string(velocity_right / (2*M_PI)) + ";\r";
    boost::asio::write(serial, boost::asio::buffer(message)); 

    return return_type::OK;
}

CallbackReturn TuwSpikeSystemInterface::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    try {
        // Define serial port settings
        std::string port_name = SERIAL_PORT;
        unsigned int baud_rate = 115200;

        // Open the serial port
        serial.open(port_name);

        // Set the baud rate
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

        // Check if we're in the bootloader or the firmware
        boost::asio::write(serial, boost::asio::buffer("version\r", 8));
        int emptydata = 0;
        int incdata = 0;
        while (true) {
            std::string line = serialReadLine();
            if (line.empty()) {
                ++emptydata;
                if (emptydata > 3) {
                    break;
                } else {
                    continue;
                }
            }

            RCUTILS_LOG_INFO_NAMED(TAG, line.c_str());
            if (line.find("Firmware version: ") != std::string::npos) {
                // firmware is already loaded
                break;
            } else if (line.find("BuildHAT bootloader version") != std::string::npos) {
                // bootloader active -> load firmware
                loadFirmware();
                break;
            } else {
                ++incdata;
                if (incdata > 5) {
                    RCUTILS_LOG_ERROR_NAMED(TAG, "Error getting BuildHAT state");
                    return CallbackReturn::ERROR;
                } else {
                    boost::asio::write(serial, boost::asio::buffer("version\r", 8));
                }
            }
        }
    
        std::string cmd = "echo 0;\r";
        boost::asio::write(serial, boost::asio::buffer(cmd));
        cmd = "plimit 1; port " + std::to_string(left_wheel_port) + "; combi 0 1 0 2 0 3 0; select 0 ; selrate 10; pid_diff " + std::to_string(left_wheel_port) + " 0 5 s2 0.0027777778 1 0.1 2.5 0 .4 0.01;\r";
        //cmd = "plimit 1; port " + std::to_string(left_wheel_port) + "; combi 0 1 0 2 0 3 0; select 0 ; selrate 10; pid " + std::to_string(left_wheel_port) + " 0 5 s2 0.0027777778 1 5 0 .1 3;\r";
        boost::asio::write(serial, boost::asio::buffer(cmd));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        cmd = "port " + std::to_string(right_wheel_port) + "; combi 0 1 0 2 0 3 0; select 0; selrate 10; pid_diff " + std::to_string(right_wheel_port) + " 0 5 s2 0.0027777778 1 0.1 2.5 0 .4 0.01;\r";
        //cmd = "port " + std::to_string(right_wheel_port) + "; combi 0 1 0 2 0 3 0; select 0; selrate 10; pid " + std::to_string(right_wheel_port) + " 0 5 s2 0.0027777778 1 5 0 .1 3;\r";
        boost::asio::write(serial, boost::asio::buffer(cmd));

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::string left_port_str = "P" + std::to_string(left_wheel_port) + "C0";
        std::string right_port_str = "P" + std::to_string(right_wheel_port) + "C0";
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
                                std::string apos = current.substr(second_space + 1, third_space - second_space - 1);

                                if (apos.size() > 0) {
                                    if (!substring.compare(left_port_str)) {
                                        try {
                                            left_wheel_offset = std::stoi(apos);
                                        } catch (std::invalid_argument const& e) {
                                            RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing pos_left: %s", e.what());
                                        }
                                    } else if (!substring.compare(right_port_str)) {
                                        try {
                                            right_wheel_offset = std::stoi(apos);
                                        } catch (std::invalid_argument const& e) {
                                            RCUTILS_LOG_ERROR_NAMED(TAG, "Error parsing pos_right: %s", e.what());
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
            left_wheel_offset = -left_wheel_offset;
        }
        if (reverse[1]) {
            right_wheel_offset = -right_wheel_offset;
        }
    
    } catch (std::runtime_error &e) {
        RCUTILS_LOG_ERROR_NAMED(TAG, "Error configuring hardware: %s",
                                e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn TuwSpikeSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    // end select
    std::string end_message = "port " + std::to_string(left_wheel_port) + "; select; set 0; port " + std::to_string(right_wheel_port) + "; select; set 0;\r";
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