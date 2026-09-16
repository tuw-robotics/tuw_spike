#include "tuw_build_hat/build_hat.hpp"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace tuw_build_hat {

static std::string current_datetime_string() {
    std::time_t now = std::time(nullptr);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

BuildHat::BuildHat() { msg(LogLevel::INFO, "BuildHat Created"); }

void BuildHat::set_device(const std::string &device_name, unsigned int baud_rate) {
    device_name_ = device_name;
    baud_rate_ = baud_rate;
}

void BuildHat::set_firmware(const std::string &firmware, const std::string &signature) {
    path_to_firmware_ = firmware;
    path_to_signature_ = signature;
}

void BuildHat::add_sensor(const std::shared_ptr<Sensor> &sensor) { ports.push_back(sensor); }
void BuildHat::set_logfile_serial(const std::string &logfile) {
    if (serial_log_.is_open()) {
        serial_log_.close();
    }
    if (!logfile.empty()) {
        serial_log_.open(logfile, std::ios::app);
        if (!serial_log_.is_open()) {
            msg(LogLevel::ERROR, "Could not open serial log file: %s", logfile.c_str());
        } else {
            serial_log_ << "--- " << current_datetime_string() << " ---" << std::endl;
        }
    }
}

void BuildHat::set_logfile_msgs(const std::string &msgfile) {
    if (msg_log_.is_open()) {
        msg_log_.close();
    }
    if (!msgfile.empty()) {
        msg_log_.open(msgfile, std::ios::app);
        if (!msg_log_.is_open()) {
            msg(LogLevel::ERROR, "Could not open msgs log file: %s", msgfile.c_str());
        } else {
            msg_log_ << "--- " << current_datetime_string() << " ---" << std::endl;
        }
    }
}
void BuildHat::set_loglevel(LogLevel loglevel) { loglevel_ = loglevel; }

void BuildHat::set_loglevel(int loglevel) { set_loglevel(static_cast<LogLevel>(loglevel)); }

int BuildHat::init() {
    msg(LogLevel::INFO, "BuildHat init");

    try {
        serial_ = std::make_unique<boost::asio::serial_port>(io_context_, device_name_);

        // Open the serial port
        serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

        msg(LogLevel::INFO, "Open %s with %i bytes/sec", device_name_.c_str(), baud_rate_);

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

            msg(LogLevel::INFO, line.c_str());
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
                    msg(LogLevel::ERROR, "Error getting BuildHAT state");
                    return ERROR;
                }
            }
        }

        int left_wheel_port = 0;
        int right_wheel_port = 1;

        std::string cmd = "echo 0;\r";
        serial_write(cmd);
        for (const auto &sensor : ports) {
            sensor->init();
            msg(LogLevel::DEBUG, sensor->cmd().c_str());
            serial_write(sensor->get_and_clear_command());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        serial_start_read();
    } catch (std::runtime_error &e) {
        msg(LogLevel::ERROR, "Error configuring hardware: %s", e.what());
        return ERROR;
    }

    return OK;
}

int BuildHat::commit() {
    for (const auto &sensor : ports) {
        if (sensor->has_cmd()) {
            msg(LogLevel::DEBUG, sensor->cmd().c_str());
            serial_write(sensor->get_and_clear_command());
        }
    }
    return OK;
}

void BuildHat::activate_with_velocity_mode(int port_id) { msg(LogLevel::INFO, "BuildHat activate_with_velocity_mode %d", port_id); }

void BuildHat::deactivate() {
    msg(LogLevel::INFO, "BuildHat deactivate");

    serial_stop_read();

    // Close serial port
    if (serial_) {
        std::string cmd;
        for (const auto &sensor : ports) {
            sensor->deactivate();
            if (sensor->has_cmd()) {
                msg(LogLevel::DEBUG, sensor->cmd().c_str());
                serial_write(sensor->get_and_clear_command());
            }
        }
        serial_write(cmd);
        serial_->close();
        serial_stop_read();
        serial_.reset();
    }

    if (serial_log_.is_open()) {
        serial_log_.close();
    }
}

double BuildHat::get_position_radian(int port_id) {
    msg(LogLevel::INFO, "BuildHat get_position_radian %d", port_id);
    return 0;
}

double BuildHat::get_velocity_radian_per_sec(int port_id) {
    msg(LogLevel::INFO, "BuildHat get_velocity_radian_per_sec %d", port_id);
    return 0;
}

void BuildHat::upload_firmware() {

    msg(LogLevel::INFO, "Upload firmware: %s", path_to_firmware_.c_str());

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
    msg(LogLevel::INFO, "Waiting %d seconds until the board is rebooted", seconds_to_reboot);
    std::this_thread::sleep_for(std::chrono::seconds(seconds_to_reboot));
}

void BuildHat::serial_write(const std::string &message) {
    boost::asio::write(*serial_, boost::asio::buffer(message));

    if (serial_log_.is_open()) {
        serial_log_ << "TX: " << message << std::endl;
    }
}

std::size_t BuildHat::serial_read(std::vector<char> &buffer, boost::system::error_code &error) {
    std::size_t bytes_read = serial_->read_some(boost::asio::buffer(buffer), error);

    if (serial_log_.is_open()) {
        serial_log_ << std::string(buffer.data(), bytes_read) << std::endl;
    }

    return bytes_read;
}

std::string BuildHat::serial_read_line() {
    boost::asio::read_until(*serial_, buf, '\n');
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);

    if (serial_log_.is_open()) {
        serial_log_ << "RX: " << line << std::endl;
    }

    return line;
}

void BuildHat::get_prompt() {
    std::string bootloader_str = "BHBL>";
    while (true) {
        std::string line = serial_read_line();
        if (line.find(bootloader_str) != std::string::npos) {
            break;
        }
    }
}

uint32_t BuildHat::checksum(const std::vector<uint8_t> &data) {
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

void BuildHat::msg(LogLevel level, const char *format, ...) {
    if (loglevel_ <= level) {
        char buffer[512];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        if (level == LogLevel::ERROR) {
            std::cerr << buffer << std::endl;
        } else {
            if (msg_log_.is_open()) {
                msg_log_ << buffer << std::endl;
            } else {
                std::cout << buffer << std::endl;
            }
        }
    }
}


void BuildHat::serial_start_read() {
    if (reading_) {
        return;
    }
    reading_ = true;
    read_thread_ = std::thread(&BuildHat::serial_read_loop, this);
}

void BuildHat::serial_stop_read() {
    if (!reading_) {
        return;
    }
    reading_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void BuildHat::serial_read_loop() {
    std::vector<char> buffer(128);
    boost::system::error_code error;

    while (reading_) {
        std::string line = serial_read_line();
        if (error) {
            if (reading_) {
                msg(LogLevel::ERROR, "Error reading from serial port: %s", error.message().c_str());
            }
            break;
        }
        (void)line;
    }
}

} // namespace tuw_build_hat
