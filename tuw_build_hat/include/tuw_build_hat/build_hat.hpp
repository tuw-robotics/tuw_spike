#ifndef BUILD_HAT_HPP
#define BUILD_HAT_HPP

#include <atomic>
#include <boost/asio.hpp>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "tuw_build_hat/sensor.hpp"

namespace tuw_build_hat {

class BuildHat {
  public:
    static constexpr int OK = 0;
    static constexpr int ERROR = 1;

    static constexpr int NUMBER_OF_PORTS = 4;
    static constexpr int LIMIT_ON = 1;
    static constexpr int LIMIT_OFF = 0;
    static constexpr int MODE_SPEED = 1;
    static constexpr int MODE_POS = 2;
    static constexpr int MODE_APOS = 3;

    
    enum class LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        ERROR = 3,
    };


  public:
    BuildHat();

    void set_device(const std::string &device_name = "/dev/ttyAMA0", unsigned int baud_rate = 115200);

    void set_firmware(const std::string &firmware, const std::string &signature);

    void set_logfile_serial(const std::string &logfile);

    void set_logfile_msgs(const std::string &logfile);

    void set_loglevel(LogLevel level);

    void set_loglevel(int level);

    void add_sensor(const std::shared_ptr<Sensor> &sensor);

    int init();

    
    int commit();

    void activate_with_velocity_mode(int port_id);

    void deactivate();

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

    // start the background thread that continuously reads from the serial port
    void serial_start_read();

    // stop the background read thread started by serial_start_read()
    void serial_stop_read();

    // body of the background read thread
    void serial_read_loop();

    // read from the serial port until "BHBL>" is read
    void get_prompt();

    // calculate the checksum for the firmware
    uint32_t checksum(const std::vector<uint8_t> &data);

    void msg(LogLevel level, const char *format, ...);

    std::string device_name_;
    unsigned int baud_rate_;
    std::string path_to_firmware_;
    std::string path_to_signature_;
    LogLevel loglevel_;
    std::ofstream serial_log_;    // if open, the serial communication is logged there
    std::ofstream msg_log_;       // if open, the msgs is logged there

    // Declare the io_context and serial port globally
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_;
    boost::asio::streambuf buf;
    std::vector<std::shared_ptr<Sensor>> ports;

    std::thread read_thread_;
    std::atomic<bool> reading_{false};
};
} // namespace tuw_build_hat

#endif
