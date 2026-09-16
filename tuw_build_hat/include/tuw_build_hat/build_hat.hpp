#ifndef TUW_BUILD_HAT__BUILD_HAT_HPP
#define TUW_BUILD_HAT__BUILD_HAT_HPP

#include <atomic>
#include <boost/asio.hpp>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "tuw_build_hat/device.hpp"

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
    static constexpr int DECODE_OK = 0;
    static constexpr int DECODE_ERROR = 1;

    
    enum class LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        ERROR = 3,
    };


  public:
    /**
     * @brief Construct a new BuildHat driver instance.
     */
    BuildHat();

    /**
     * @brief Configure the serial device used to talk to the Build HAT.
     * @param device_name path to the serial device, e.g. "/dev/ttyAMA0"
     * @param baud_rate baud rate to open the serial port with
     */
    void set_device(const std::string &device_name = "/dev/ttyAMA0", unsigned int baud_rate = 115200);

    /**
     * @brief Set the paths to the firmware and signature files to upload if needed.
     * @param firmware path to the firmware binary
     * @param signature path to the signature binary
     */
    void set_firmware(const std::string &firmware, const std::string &signature);

    /**
     * @brief Enable or disable logging of the raw serial communication to a file.
     * @param logfile path to the log file, or empty to disable logging
     */
    void set_logfile_serial(const std::string &logfile);

    /**
     * @brief Enable or disable logging of driver messages to a file.
     * @param logfile path to the log file, or empty to log to std::cout/std::cerr instead
     */
    void set_logfile_msgs(const std::string &logfile);

    /**
     * @brief Set the minimum severity level of messages that get logged.
     * @param level minimum LogLevel to log
     */
    void set_loglevel(LogLevel level);

    /**
     * @brief Set the minimum severity level of messages that get logged.
     * @param level minimum log level as an integer (0 = DEBUG, 1 = INFO, 2 = WARNING, 3 = ERROR)
     */
    void set_loglevel(int level);

    /**
     * @brief Register a device (e.g. a Motor) attached to a Build HAT port.
     * @param device the device to add
     */
    void add_device(const std::shared_ptr<Device> &device);

    /**
     * @brief Unregister the device attached to a given Build HAT port, if any.
     * @param port id of the port whose device should be removed
     */
    void remove_device(unsigned int port);

    /**
     * @brief Open the serial port, ensure the firmware is loaded, and initialize all registered devices.
     * @return OK on success, ERROR otherwise
     * @pre set_device() and set_firmware() must be called beforehand.
     * @pre set_loglevel(), set_logfile_serial() and set_logfile_msgs() are optional and, if used, should be called beforehand too.
     */
    int init();

    /**
     * @brief Send any pending commands queued up by the registered devices to the Build HAT.
     * @return OK on success
     * @pre commands on the registed devices
     */
    int commit();

    /**
     * @brief Activate a port in velocity control mode.
     * @param port_id id of the port to activate
     */
    void activate_with_velocity_mode(int port_id);

    /**
     * @brief Deactivate all devices, stop the read thread, and close the serial port.
     */
    void deactivate();

    /**
     * @brief Get the last known position of a port.
     * @param port_id id of the port to query
     * @return position in radians
     */
    double get_position_radian(int port_id);

    /**
     * @brief Get the last known velocity of a port.
     * @param port_id id of the port to query
     * @return velocity in radians per second
     */
    double get_velocity_radian_per_sec(int port_id);

  private:
    /**
     * @brief Upload the firmware and signature to the Build HAT bootloader and reboot it.
     */
    void upload_firmware();

    /**
     * @brief Write a string to the serial port, logging it if serial logging is enabled.
     * @param message the string to write
     */
    void serial_write(const std::string &message);

    /**
     * @brief Read some data from the serial port into buffer.
     * @param buffer buffer to read into
     * @param error set to the resulting error code, if any
     * @return the number of bytes read
     */
    std::size_t serial_read(std::vector<char> &buffer, boost::system::error_code &error);

    /**
     * @brief Read a single line from the serial port.
     * @param error set to the resulting error code, if any
     * @return the line that was read, without the trailing newline
     */
    std::string serial_read_line(boost::system::error_code &error);

    /**
     * @brief Start the background thread that continuously reads from the serial port.
     */
    void serial_start_read();

    /**
     * @brief Stop the background read thread started by serial_start_read().
     */
    void serial_stop_read();

    /**
     * @brief Body of the background read thread: reads lines and forwards them to the registered devices.
     */
    void serial_read_loop();

    /**
     * @brief Read from the serial port until the bootloader prompt "BHBL>" is seen.
     * @return true once the prompt is found, false on a read error
     */
    bool get_prompt();

    /**
     * @brief Calculate the checksum expected by the Build HAT bootloader for a firmware image.
     * @param data the firmware bytes to checksum
     * @return the computed checksum
     */
    uint32_t checksum(const std::vector<uint8_t> &data);

    /**
     * @brief Log a formatted message if its level is at or above the configured log level.
     * @param level severity of the message
     * @param format printf-style format string
     */
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
    std::vector<std::shared_ptr<Device>> devices_;

    std::thread read_thread_;
    std::atomic<bool> reading_{false};
};
} // namespace tuw_build_hat

#endif // TUW_BUILD_HAT__BUILD_HAT_HPP
