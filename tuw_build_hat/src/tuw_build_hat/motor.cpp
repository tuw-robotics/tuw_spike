#include "tuw_build_hat/motor.hpp"
#include "tuw_build_hat/build_hat.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace tuw_build_hat {

Motor::Motor(unsigned int port) : Device(port) {
    plimit(1.0);
}

const std::string &Motor::info() const {
    static const std::string name = "Motor";
    return name;
}

void Motor::init() {
    cmd_ += std::format("plimit {}; ", plimit());
    cmd_ += std::format("port {}; ", port_);
    cmd_ += std::format("combi 0 {} 0 {} 0 {} 0; ", BuildHat::MODE_SPEED, BuildHat::MODE_POS, BuildHat::MODE_APOS);
    cmd_ += std::format("select 0; ");             // streams the whole payload
    cmd_ += std::format("selrate {}; ", selrate_); // updates rate
    cmd_ += std::format("\r");

    serial_serach_str_ = "P" + std::to_string(port_) + "C0";
}

void Motor::set_target_radian_per_sec(double speed, double kp, double ki, double kd) {

    speed_target_ = speed;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    int percent = Device::convert_speed_to_percent(speed_target_, MAX_RAD_PER_SEC);

    int pvport = port_;           // Port to read the process variable from
    int pvmode = 0;               // Mode to read from. Here that's the combi mode 0 you selected.
    int pvoffset = 0;             // Byte offset in that mode's data. Speed is at byte 0.
    const char *pvformat = "s1";  // Signed byte. Speed is an integer from −100 to +100 %.
    double pvscale = 1.0;         // Multiplier applied to the raw value. Use it to convert % into rad/s.
    int pvunwrap = 0;             // 0 means no phase unwrapping, which is what you want for speed
    kp_ = kp;                     // Proportional gain (error → drive level from −1 to 1)
    ki_ = ki;                     // Integral gain. The time base is 1 s.
    kd_ = kd;                     // Derivative gain. The time base is 1 s.
    int windup = 100;             // Absolute clamp on the integrator
    double deadzone = 0.01;       // Error band (rad/s) where the controller does not react

    cmd_ += std::format("port {}; ", port_);
    cmd_ += std::format("pid {} {} {} {} {} {} {} {} {} {} {}; ", port_, pvmode, pvoffset, pvformat, pvscale, pvunwrap, kp_, ki_, kd_, windup, deadzone); // updates rate
    cmd_ += std::format("set {}; ", percent);
    cmd_ += std::format("\r");
}

void Motor::deactivate() {
    cmd_ += std::format("port {};", port_);
    cmd_ += std::format("coast;");
    cmd_ += std::format("select");
    cmd_ += std::format("\r");
}

bool Motor::is_feedback(const std::string &line) const {

    if (line.find(serial_serach_str_) != std::string_view::npos) {
        return true;
    }
    return false;
}

int Motor::decode(const std::string &line) {

    int port = -1;
    int combi = -1;
    int speed_percent = 0;
    long cumulative_deg = 0;
    int absolute_deg = 0;
    // %d / %ld accept leading '+' and '-', spaces are skipped automatically
    int n = std::sscanf(line.c_str(), " P%dC%d: %d %ld %d", &port, &combi, &speed_percent, &cumulative_deg, &absolute_deg);

    if (n != 5 || static_cast<int>(port_) != port) {
        return BuildHat::DECODE_ERROR;
    }

    rps_ = Device::convert_percent_to_speed(speed_percent, MAX_RAD_PER_SEC);
    cumulative_rad_ = Device::convert_deg_to_rad(cumulative_deg);
    absolute_rad_ = Device::normalize_rad(Device::convert_deg_to_rad(absolute_deg));

    std::cout << port_ << ": " << rps_ << ", " << cumulative_rad_ << ", " << absolute_rad_ << std::endl;
    return BuildHat::DECODE_OK;
}
} // namespace tuw_build_hat
