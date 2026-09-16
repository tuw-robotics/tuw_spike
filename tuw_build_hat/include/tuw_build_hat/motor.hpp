#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "tuw_build_hat/sensor.hpp"

namespace tuw_build_hat {

class Motor : public Sensor {
  static constexpr double kMaxRadPerSec = 18.5;
  public:
    explicit Motor(unsigned int id);
    virtual ~Motor() = default;

    void init() override;
    void set_target_radian_per_sec(double rps, double kp = 0.003, double ki= 0.01, double kd = 0.0);
    void deactivate() override;

  private:
    double target_rps_;
    double kp_;
    double ki_;
    double kd_;
};

} // namespace tuw_build_hat

#endif
