#include "tuw_build_hat/motor.hpp"
#include "tuw_build_hat/build_hat.hpp"
#include <algorithm>
#include <cmath>

namespace tuw_build_hat {

Motor::Motor(unsigned int id) : Sensor(id) {
    plimit(0.7);
}

void Motor::init() {
    cmd_ += std::format("plimit {}; ", plimit());
    cmd_ += std::format("port {}; ", id_);
    cmd_ += std::format("combi 0 {} 0 {} 0 {} 0; ", BuildHat::MODE_SPEED, BuildHat::MODE_POS, BuildHat::MODE_APOS);
    cmd_ += std::format("select 0; ");                                               // streams the whole payload
    cmd_ += std::format("selrate {}; ", selrate_);                                   // updates rate
    cmd_ += std::format("\r");
      
}

void Motor::set_target_radian_per_sec(double rps, double kp, double ki, double kd) {

    target_rps_ = rps;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    double percent = target_rps_ /  kMaxRadPerSec * 100.0;
    percent = std::clamp(percent, -100.0, 100.0);
    int setpoint = static_cast<int>(std::lround(percent));
    cmd_ += std::format("port {}; ", id_);
    cmd_ += std::format("pid {} 0 0 s1 1 0 {} {} {} 100 0.01; ", id_, kp_, ki_, kd_); // updates rate
    cmd_ += std::format("set {}; ", setpoint);
    cmd_ += std::format("\r");
}

void Motor::deactivate(){
    cmd_ += std::format("port {};", id_);
    cmd_ += std::format("coast;", id_);
    cmd_ += std::format("select");
    cmd_ += std::format("\r");
}

} // namespace tuw_build_hat
