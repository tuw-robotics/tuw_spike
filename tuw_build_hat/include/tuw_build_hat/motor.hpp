#ifndef TUW_BUILD_HAT__MOTOR_HPP
#define TUW_BUILD_HAT__MOTOR_HPP

#include "tuw_build_hat/device.hpp"

namespace tuw_build_hat {

/**
 * @brief Device implementation for a Build HAT motor port, driven in closed-loop speed mode.
 *
 * A Motor is set up in "combi" mode 0, streaming speed, cumulative position and absolute
 * position feedback for its port every selrate() milliseconds. set_target_radian_per_sec()
 * drives the port with the Build HAT firmware's onboard PID speed controller, and decode()
 * parses the resulting feedback lines back into rad/s and radians so they can be read back
 * with get_velocity_radian_per_sec() / get_position_radian().
 */
class Motor : public Device {
  public:
    /**
     * @brief Maximum motor speed, in radians per second, used to scale between rad/s and the
     *        -100..100% "set" value the Build HAT firmware expects.
     */
    static constexpr double MAX_RAD_PER_SEC = 18.5;

    /**
     * @brief Construct a Motor attached to a given Build HAT port.
     * @param port id of the Build HAT port (0-3) the motor is plugged into
     */
    explicit Motor(unsigned int port);

    virtual ~Motor() = default;

    /**
     * @brief Get a short human-readable name for this device, used in log messages.
     * @return the string "Motor"
     */
    const std::string &info() const override;

    /**
     * @brief Queue a command that drives the motor to a target speed using the Build HAT's
     *        onboard PID speed controller.
     *
     * Converts speed into a -100..100% setpoint (relative to MAX_RAD_PER_SEC) and appends a
     * "pid" + "set" command to the pending command buffer; call BuildHat::commit() to
     * actually send it to the Build HAT.
     *
     * @param speed target speed in radians per second (positive/negative for direction)
     * @param kp proportional gain of the onboard speed controller
     * @param ki integral gain of the onboard speed controller (time base 1 s)
     * @param kd derivative gain of the onboard speed controller (time base 1 s)
     */
    void set_target_radian_per_sec(double speed, double kp = 0.003, double ki = 0.01, double kd = 0.0);

    /**
     * @brief Get the last known absolute position of the motor, as decoded from feedback.
     * @return position in radians, normalized to [-pi, pi)
     */
    double get_position_radian() const {
      return absolute_rad_;
    }

    /**
     * @brief Get the last known velocity of the motor, as decoded from feedback.
     * @return velocity in radians per second
     */
    double get_velocity_radian_per_sec() const {
      return speed_current_;
    }

    /**
     * @brief Get the target velocity last requested via set_target_radian_per_sec().
     * @return velocity in radians per second
     */
    double get_velocity_radian_per_sec_target() const {
      return speed_target_;
    }

  private:
    /**
     * @brief Queue the commands that arm the port in combi speed/position/absolute-position
     *        mode and start streaming feedback at selrate() ms.
     */
    void init() override;

    /**
     * @brief Parse a feedback line for this motor's port and update the cached speed/position.
     * @param msg one line read from the serial port
     * @return BuildHat::DECODE_OK on success, BuildHat::DECODE_ERROR if the line could not be parsed
     */
    int decode(const std::string &msg) override;

    /**
     * @brief Check whether a line read from the serial port is feedback for this motor's port.
     * @param msg one line read from the serial port
     * @return true if msg starts with this motor's port/combi prefix (e.g. "P0C0")
     */
    bool is_feedback(const std::string &msg) const override;

    /**
     * @brief Queue the command that coasts the motor and deselects its port.
     */
    void deactivate() override;

  private:
    double speed_target_;   ///< target speed, last requested via set_target_radian_per_sec() [rad/s]
    double speed_current_;  ///< current speed, as decoded from feedback [rad/s]
    double cumulative_rad_; ///< cumulative (unwrapped) position, as decoded from feedback [rad]
    double absolute_rad_;   ///< absolute position normalized to [-pi, pi), as decoded from feedback [rad]
    double kp_;              ///< proportional gain last passed to set_target_radian_per_sec()
    double ki_;              ///< integral gain last passed to set_target_radian_per_sec()
    double kd_;              ///< derivative gain last passed to set_target_radian_per_sec()
    std::string serial_serach_str_; ///< "P<port>C0" prefix used by is_feedback() to recognize this motor's lines
};

} // namespace tuw_build_hat

#endif // TUW_BUILD_HAT__MOTOR_HPP
