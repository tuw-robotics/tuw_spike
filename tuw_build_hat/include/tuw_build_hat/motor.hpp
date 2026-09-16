#ifndef TUW_BUILD_HAT__MOTOR_HPP
#define TUW_BUILD_HAT__MOTOR_HPP

#include "tuw_build_hat/device.hpp"

namespace tuw_build_hat {

class Motor : public Device {
  public:
    static constexpr double MAX_RAD_PER_SEC = 18.5;

    explicit Motor(unsigned int port);
    virtual ~Motor() = default;

    const std::string &info() const override;
    void set_target_radian_per_sec(double speed, double kp = 0.003, double ki = 0.01, double kd = 0.0);

  private:
    void init() override;
    int decode(const std::string &msg) override;
    bool is_feedback(const std::string &msg) const override;
    void deactivate() override;

  private:
    double speed_target_; // target speed [rad per seconds]
    double rps_;          // current speed [rad per seconds]
    double cumulative_rad_; // cumulative position in [rad]
    double absolute_rad_; // absolute position of the motor [rad]
    double kp_;
    double ki_;
    double kd_;
    std::string serial_serach_str_;
};

} // namespace tuw_build_hat

#endif // TUW_BUILD_HAT__MOTOR_HPP
