#ifndef BUILD_HAT_DRIVER_HPP
#define BUILD_HAT_DRIVER_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace tuw_spike_control {

class Port {
  public:
    explicit Port(unsigned int id) : id_(id) {}
    virtual ~Port() = default;

    virtual void init() = 0;

  protected:
    unsigned int id_;
};

class Motor : public Port {

};


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

    void set_logfile(const std::string &logfile);

    int init();

    void activate_with_velocity_mode(int port_id);

    void deactivate();

    void set_target_velocity_radian_per_sec(int port_id, double command);

    double get_position_radian(int port_id);

    double get_velocity_radian_per_sec(int port_id);

  private:
    void upload_firmware();

    // write a string to the serial port
    void serial_write(const std::string &message);

    // read some data from the serial port into buffer, returns the number of bytes read
    std::size_t serial_read(std::vector<char> &buffer, boost::system::error_code &error);

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
    std::ofstream serial_log_;    // if open, the serial communication is logged there

    // Declare the io_context and serial port globally
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_;
    boost::asio::streambuf buf;
};
} // namespace tuw_spike_control

#endif
