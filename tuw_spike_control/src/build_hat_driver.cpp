#include "tuw_spike_control/build_hat_driver.hpp"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <ament_index_cpp/get_package_share_path.hpp>

namespace tuw_spike_control {

BuildHatDriver::BuildHatDriver() {
    print_info("BuildHatDriver Created");
    set_logfile("/tmp/spike_serial.log");
}

void BuildHatDriver::set_device(const std::string &device_name, unsigned int baud_rate) {
    device_name_ = device_name;
    baud_rate_ = baud_rate;
}

void BuildHatDriver::set_firmware(const std::string &firmware, const std::string &signature) {
    path_to_firmware_ = firmware;
    path_to_signature_ = signature;
}

void BuildHatDriver::set_logfile(const std::string &logfile) {
    if (serial_log_.is_open()) {
        serial_log_.close();
    }
    if (!logfile.empty()) {
        serial_log_.open(logfile, std::ios::trunc);
        if (!serial_log_.is_open()) {
            print_error("Could not open serial log file: %s", logfile.c_str());
        }
    }
}

int BuildHatDriver::init() {
    print_info("BuildHatDriver init");

    try {
        serial_ = std::make_unique<boost::asio::serial_port>(io_context_, device_name_);

        // Open the serial port
        serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

        print_info("Open %s with %i bytes/sec", device_name_.c_str(), baud_rate_);

        // Check if we're in the bootloader or the firmware
        int emptydata = 0;
        int incdata = 0;
        bool firmeware_ready = false;
        serial_write("version\r");
        while (firmeware_ready == false) {
            std::string line = serial_read_line();
            if (line.empty()) {
                ++emptydata;
                if (emptydata > 3) {
                    break;
                } else {
                    continue;
                }
            }

            print_info(line.c_str());
            if (line.find("Firmware version: ") != std::string::npos) {
                // firmware is already loaded
                break;
            } else if (line.find("BuildHAT bootloader version") != std::string::npos) {
                // bootloader active -> upload firmware
                upload_firmware();
                break;
            } else {
                ++incdata;
                if (incdata > 5) {
                    print_error("Error getting BuildHAT state");
                    return ERROR;
                }
            }
        }

        int left_wheel_port = 0;
        int right_wheel_port = 1;

        int selrate = 10; // sets how often the Build HAT firmware sends unsolicited sensor/motor updates for the currently selected mode on that port [ms]

        std::string cmd = "echo 0;\r";
        serial_write(cmd);
        cmd = "plimit 1; port " + std::to_string(left_wheel_port) + "; combi 0 1 0 2 0 3 0; select 0 ; selrate  " + std::to_string(selrate) + "; pid_diff " + std::to_string(left_wheel_port) + " 0 5 s2 0.0027777778 1 0.1 2.5 0 .4 0.01;\r";
        print_info(cmd.c_str());
        cmd.clear();
        cmd += std::format("plimit {};", LIMIT_ON);
        cmd += std::format("port {};", right_wheel_port);
        cmd += std::format("combi 0 {} 0 {} 0 {} 0;", MODE_SPEED, MODE_POS, MODE_APOS);
        cmd += std::format("select 0;");  // streams the whole payload
        cmd += std::format("selrate {};", selrate);  // updates rate
        cmd += std::format("pid_diff {} 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;", right_wheel_port );  // updates rate
        cmd += std::format("\r");

        print_info(cmd.c_str());
        print_info("xx");

        serial_write(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        cmd = "plimit 1; port " + std::to_string(right_wheel_port) + "; combi 0 1 0 2 0 3 0; select 0; selrate 1; pid_diff " + std::to_string(right_wheel_port) + " 0 5 s2 0.0027777778 1 0.1 2.5 0 .4 0.01;\r";
        serial_write(cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::string left_port_str = "P" + std::to_string(left_wheel_port) + "C0";
        std::string right_port_str = "P" + std::to_string(right_wheel_port) + "C0";
        // Buffer to store incoming data
        std::vector<char> buffer(128); // Adjust size as needed
        // Read data from serial port
        boost::system::error_code error;

        std::size_t bytes_read = 128;

        std::string return_msg;
        int left_wheel_offset = 0;
        int right_wheel_offset = 0;

        while (bytes_read == 128) {
            bytes_read = serial_read(buffer, error);
            return_msg.append(buffer.data(), bytes_read);

            std::string current;
            if (error) {
                print_error("Error reading from serial port: %s", error.message().c_str());
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
                                        } catch (std::invalid_argument const &e) {
                                            print_error("Error parsing pos_left: %s", e.what());
                                        }
                                    } else if (!substring.compare(right_port_str)) {
                                        try {
                                            right_wheel_offset = std::stoi(apos);
                                        } catch (std::invalid_argument const &e) {
                                            print_error("Error parsing pos_right: %s", e.what());
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
        print_info(return_msg.c_str());
    } catch (std::runtime_error &e) {
        print_error("Error configuring hardware: %s", e.what());
        return ERROR;
    }


    return OK;
}

void BuildHatDriver::activate_with_velocity_mode(int port_id) { print_info("BuildHatDriver activate_with_velocity_mode"); }

void BuildHatDriver::deactivate() {
    print_info("BuildHatDriver deactivate");

    // Close serial port
    if (serial_) {
        std::string cmd;
        for (unsigned port_id = 0; port_id <= NUMBER_OF_PORTS; ++port_id) {
            cmd += std::format("port {};", port_id);
            cmd += std::format("select ;");
            cmd += std::format("set 0;");
        }
        cmd += std::format("\r;");
        serial_write(cmd);
        serial_->close();
        serial_.reset();
    }

    if (serial_log_.is_open()) {
        serial_log_.close();
    }
}

void BuildHatDriver::set_target_velocity_radian_per_sec(int port_id, double command) {

    double target_left = command;
    std::string message = "port " + std::to_string(port_id) + "; plimit 1; set " + std::to_string(target_left) + ";\r";

    print_info("message: %s", message.c_str());
    serial_write(message);
}

double BuildHatDriver::get_position_radian(int dxl_id) { return 0; }

double BuildHatDriver::get_velocity_radian_per_sec(int dxl_id) { return 0; }

void BuildHatDriver::upload_firmware() {

    print_info("Upload firmware: %s", path_to_firmware_.c_str());

    std::ifstream file_firmware(path_to_firmware_, std::ios::binary);
    std::vector<unsigned char> firmware = std::vector<unsigned char>(std::istreambuf_iterator<char>(file_firmware), {});

    std::ifstream file_signature(path_to_signature_, std::ios::binary);
    std::vector<unsigned char> signature = std::vector<unsigned char>(std::istreambuf_iterator<char>(file_signature), {});

    // clear current image
    serial_write("clear\r");
    get_prompt();

    // write firmware to serial port
    std::string load_command = "load " + std::to_string(std::filesystem::file_size(path_to_firmware_)) + " " + std::to_string(checksum(firmware)) + "\r";
    serial_write(load_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_write("\x02");
    boost::asio::write(*serial_, boost::asio::buffer(firmware));
    serial_write("\x03\r");
    get_prompt();

    // write signature to serial port
    std::string signature_command = "signature " + std::to_string(std::filesystem::file_size(path_to_signature_)) + "\r";
    serial_write(signature_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_write("\x02");
    boost::asio::write(*serial_, boost::asio::buffer(signature));
    serial_write("\x03\r");
    get_prompt();

    serial_write("reboot\r");

    // waiting some seconds until reboot is finished
    int seconds_to_reboot = 10;
    print_info("Waiting %d seconds until the board is rebooted", seconds_to_reboot);
    std::this_thread::sleep_for(std::chrono::seconds(seconds_to_reboot));
}

void BuildHatDriver::serial_write(const std::string &message) {
    boost::asio::write(*serial_, boost::asio::buffer(message));

    if (serial_log_.is_open()) {
        serial_log_ << "TX: " << message << std::endl;
    }
}

std::size_t BuildHatDriver::serial_read(std::vector<char> &buffer, boost::system::error_code &error) {
    std::size_t bytes_read = serial_->read_some(boost::asio::buffer(buffer), error);

    if (serial_log_.is_open()) {
        serial_log_ << std::string(buffer.data(), bytes_read) << std::endl;
    }

    return bytes_read;
}

std::string BuildHatDriver::serial_read_line() {
    boost::asio::read_until(*serial_, buf, '\n');
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);

    if (serial_log_.is_open()) {
        serial_log_ << "RX: " << line << std::endl;
    }

    return line;
}

void BuildHatDriver::get_prompt() {
    std::string bootloader_str = "BHBL>";
    while (true) {
        std::string line = serial_read_line();
        if (line.find(bootloader_str) != std::string::npos) {
            break;
        }
    }
}

uint32_t BuildHatDriver::checksum(const std::vector<uint8_t> &data) {
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

void BuildHatDriver::print_info(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    std::cout << buffer << std::endl;
}

void BuildHatDriver::print_error(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    std::cerr << buffer << std::endl;
}

} // namespace tuw_spike_control
