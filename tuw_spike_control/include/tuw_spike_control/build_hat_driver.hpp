#ifndef BUILD_HAT_DRIVER_HPP
#define BUILD_HAT_DRIVER_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tuw_spike_control {

class BuildHatDriver {
    static constexpr int OK = 0;
    static constexpr int ERROR = 1;

    static constexpr int NUMBER_OF_PORTS = 4;
    static constexpr int LIMIT_ON = 1;
    static constexpr int LIMIT_OFF = 0;
    static constexpr int MODE_SPEED = 1;
    static constexpr int MODE_POS = 2;
    static constexpr int MODE_APOS = 3;
  public:
    BuildHatDriver();

    void set_device(const std::string &device_name = "/dev/ttyAMA0", unsigned int baud_rate = 115200);

    void set_firmware(const std::string &firmware, const std::string &signature);

    int init();

    void activate_with_velocity_mode(int port_id);

    void deactivate();

    void set_target_velocity_radian_per_sec(int port_id, double command);

    double get_position_radian(int port_id);

    double get_velocity_radian_per_sec(int port_id);

  private:
    void upload_firmware();

    // read a line from the serial port
    std::string serial_read_line();

    // read from the serial port until "BHBL>" is read
    void get_prompt();

    // calculate the checksum for the firmware
    uint32_t checksum(const std::vector<uint8_t> &data);

    void print_info(const char *format, ...);

    void print_error(const char *format, ...);

    std::string device_name_;
    unsigned int baud_rate_;
    std::string path_to_firmware_;
    std::string path_to_signature_;

    // Declare the io_context and serial port globally
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_;
    boost::asio::streambuf buf;
};
} // namespace tuw_spike_control

#endif
