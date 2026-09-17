#ifndef TUW_BUILD_HAT__DEVICE_HPP
#define TUW_BUILD_HAT__DEVICE_HPP

#include <string>

namespace tuw_build_hat {

/**
 * @brief Abstract base for anything attached to a Build HAT port (e.g. Motor).
 *
 * A Device owns a queue of pending command text (cmd_) that BuildHat::commit() drains via
 * get_and_clear_command() and writes to the serial port, and is asked to decode() each
 * feedback line the port streams back once is_feedback() recognizes it as its own. Derived
 * classes implement the pure virtual hooks to build port-specific setup/teardown commands
 * and to interpret their own feedback format; this base class provides the port id, the
 * shared command buffer, and the unit-conversion helpers commonly needed to do so.
 */
class Device {
  public:
    /**
     * @brief Construct a Device attached to a given Build HAT port.
     * @param port id of the Build HAT port (0-3) the device is plugged into
     */
    explicit Device(unsigned int port);

    virtual ~Device() = default;

    /**
     * @brief Get a short human-readable name for this device, used in log messages.
     * @return a static string identifying the device type, e.g. "Motor"
     */
    virtual const std::string &info() const = 0;

    /**
     * @brief Queue the commands needed to arm this device's port for use.
     *
     * Called once by BuildHat::init() for every registered device.
     */
    virtual void init() = 0;

    /**
     * @brief Queue the commands needed to bring this device's port to a safe, deselected state.
     *
     * Called once by BuildHat::deactivate() for every registered device.
     */
    virtual void deactivate() = 0;

    /**
     * @brief Parse a feedback line recognized (via is_feedback()) as belonging to this device.
     * @param msg one line read from the serial port
     * @return BuildHat::DECODE_OK on success, BuildHat::DECODE_ERROR if the line could not be parsed
     */
    virtual int decode(const std::string &msg) = 0;

    /**
     * @brief Check whether a line read from the serial port is feedback for this device.
     * @param msg one line read from the serial port
     * @return true if msg should be passed to decode()
     */
    virtual bool is_feedback(const std::string &msg) const = 0;

    /**
     * @brief Get the id of the Build HAT port this device is attached to.
     * @return the port id passed to the constructor
     */
    unsigned int port();

    /**
     * @brief Set how often the firmware sends unsolicited feedback updates for this port.
     * @param ms update period in milliseconds
     */
    void selrate(int ms);

    /**
     * @brief Get the configured feedback update period.
     * @return update period in milliseconds
     */
    int selrate() const;

    /**
     * @brief Set the power limit applied to this port.
     * @param power power limit as a fraction, from 0.0 to 1.0
     */
    void plimit(double power);

    /**
     * @brief Get the configured power limit.
     * @return power limit as a fraction, from 0.0 to 1.0
     */
    double plimit() const;

    /**
     * @brief Peek at the currently queued, not-yet-sent command text.
     * @return the pending command string, without clearing it
     */
    const std::string &cmd() const;

    /**
     * @brief Take ownership of the currently queued command text and clear the queue.
     * @return the command string that was pending
     */
    std::string get_and_clear_command();

    /**
     * @brief Check whether there is a queued, not-yet-sent command.
     * @return true if cmd() is non-empty
     */
    bool has_cmd() const;

    /**
     * @brief Convert a speed in radians per second to the -100..100% "set" value the
     *        Build HAT firmware expects, clamped to that range.
     * @param rad_per_sec speed to convert
     * @param max_rad_per_sec speed, in radians per second, that maps to +-100%
     * @return the equivalent percentage, rounded to the nearest integer
     */
    static int convert_speed_to_percent(double rad_per_sec, double max_rad_per_sec);

    /**
     * @brief Inverse of convert_speed_to_percent(): convert a -100..100% value back to rad/s.
     * @param percent value to convert, clamped to -100..100 first
     * @param max_rad_per_sec speed, in radians per second, that +-100% maps to
     * @return the equivalent speed in radians per second
     */
    static double convert_percent_to_speed(int percent, double max_rad_per_sec);

    /**
     * @brief Convert an angle in degrees to radians.
     * @param deg angle in degrees
     * @return angle in radians
     */
    static double convert_deg_to_rad(double deg);

    /**
     * @brief Convert an angle in radians to degrees, rounded to the nearest integer.
     * @param rad angle in radians
     * @return angle in degrees, wrapped modulo 360 but not yet normalized (see normalize_deg())
     */
    static int convert_rad_to_deg(double rad);

    /**
     * @brief Normalize an angle in degrees into the range [-180, 179].
     * @param deg angle in degrees, any magnitude
     * @return the equivalent angle in [-180, 179]
     */
    static int normalize_deg(int deg);

    /**
     * @brief Normalize an angle in radians into the range [-pi, pi).
     * @param rad angle in radians, any magnitude
     * @return the equivalent angle in [-pi, pi)
     */
    static double normalize_rad(double rad);

  protected:
    unsigned int port_;    ///< id of the Build HAT port this device is attached to
    unsigned int selrate_; ///< how often the firmware sends unsolicited feedback updates for this port [ms]
    double plimit_;        ///< power limit applied to this port, as a fraction from 0.0 to 1.0
    std::string cmd_;      ///< command text queued up for the next commit(), see get_and_clear_command()
};

} // namespace tuw_build_hat

#endif // TUW_BUILD_HAT__DEVICE_HPP
