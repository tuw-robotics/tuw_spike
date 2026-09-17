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

/**
 * @brief Driver for a Raspberry Pi Build HAT, talking to it over a direct serial link.
 *
 * Owns the serial connection, uploads firmware to the bootloader if needed, and dispatches
 * commands and feedback for a set of registered Device instances (e.g. Motor). Typical usage:
 * construct a BuildHat, call set_device()/set_firmware() (and optionally set_loglevel() and the
 * set_logfile_*() setters), add_device() one Device per Build HAT port, call init(), then drive
 * devices and call commit() to flush their queued commands, finishing with deactivate().
 */
class BuildHat {
  public:
    static constexpr int OK = 0;    ///< return value of init()/commit() on success
    static constexpr int ERROR = 1; ///< return value of init() on failure

    static constexpr int NUMBER_OF_PORTS = 4; ///< number of physical ports on a Build HAT
    static constexpr int LIMIT_ON = 1;        ///< value passed to the "plimit"/"port_plimit" command to enable the power limit
    static constexpr int LIMIT_OFF = 0;       ///< value passed to the "plimit"/"port_plimit" command to disable the power limit
    static constexpr int MODE_SPEED = 1;      ///< Build HAT combi-mode index for speed feedback
    static constexpr int MODE_POS = 2;        ///< Build HAT combi-mode index for cumulative position feedback
    static constexpr int MODE_APOS = 3;       ///< Build HAT combi-mode index for absolute position feedback
    static constexpr int DECODE_OK = 0;       ///< return value of Device::decode() when a feedback line was parsed successfully
    static constexpr int DECODE_ERROR = 1;    ///< return value of Device::decode() when a feedback line could not be parsed

    /**
     * @brief Severity levels used by set_loglevel() and msg() to filter log output.
     */
    enum class LogLevel {
        DEBUG = 0,   ///< verbose diagnostic messages
        INFO = 1,    ///< normal operational messages
        WARNING = 2, ///< unexpected but recoverable conditions
        ERROR = 3,   ///< failures
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
     *
     * Call this after issuing commands on one or more registered devices (e.g.
     * Motor::set_target_radian_per_sec()) to actually write them to the serial port.
     * @return OK on success
     */
    int commit();

    /**
     * @brief Activate a port in velocity control mode.
     * @param port_id id of the port to activate
     * @note currently a stub: only logs the request, does not yet send a command
     */
    void activate_with_velocity_mode(int port_id);

    /**
     * @brief Deactivate all devices, stop the read thread, and close the serial port.
     */
    void deactivate();


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

    std::string device_name_;         ///< path to the serial device, set via set_device()
    unsigned int baud_rate_;          ///< baud rate to open the serial port with, set via set_device()
    std::string path_to_firmware_;    ///< path to the firmware binary, set via set_firmware()
    std::string path_to_signature_;   ///< path to the signature binary, set via set_firmware()
    LogLevel loglevel_;                ///< minimum severity level logged by msg(), set via set_loglevel()
    std::ofstream serial_log_;        ///< if open, the raw serial communication is logged here
    std::ofstream msg_log_;           ///< if open, driver messages are logged here instead of std::cout/std::cerr

    boost::asio::io_context io_context_;               ///< io_context backing the serial port
    std::unique_ptr<boost::asio::serial_port> serial_;  ///< the open serial connection to the Build HAT, null until init()
    boost::asio::streambuf buf;                         ///< streambuf used to buffer incoming data for serial_read_line()
    std::vector<std::shared_ptr<Device>> devices_;      ///< devices registered via add_device(), one per Build HAT port

    std::thread read_thread_;         ///< background thread running serial_read_loop(), started by serial_start_read()
    std::atomic<bool> reading_{false}; ///< true while the background read thread should keep running
};
} // namespace tuw_build_hat

#endif // TUW_BUILD_HAT__BUILD_HAT_HPP
