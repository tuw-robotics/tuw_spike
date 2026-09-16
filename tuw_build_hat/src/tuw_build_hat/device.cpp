#include "tuw_build_hat/device.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace tuw_build_hat {

Device::Device(unsigned int port) : port_(port), selrate_(100) {}

    unsigned int Device::port(){
        return port_;
    }
    void Device::selrate(int ms){
        selrate_ = ms;
    }
    int Device::selrate() const{
        return selrate_;
    }

    void Device::plimit(double power){
        plimit_ = power;
    }
    double Device::plimit() const{
        return plimit_;
    }

    const std::string &Device::cmd() const{
      return cmd_;
    }

    std::string Device::get_and_clear_command(){
      std::string ret(cmd_);
      cmd_.clear();
      return ret;
    }

    bool Device::has_cmd() const{
      return !cmd_.empty();
    }

    int Device::convert_speed_to_percent(double rad_per_sec, double max_rad_per_sec) {
        double percent = std::clamp(rad_per_sec / max_rad_per_sec * 100.0, -100.0, 100.0);
        return static_cast<int>(std::lround(std::clamp(percent, -100.0, 100.0)));
    }

    double Device::convert_percent_to_speed(int percent, double max_rad_per_sec) {
        return std::clamp(static_cast<double>(percent), -100.0, 100.0) / 100.0 * max_rad_per_sec;
    }

    double Device::convert_deg_to_rad(double deg) {
        return deg * M_PI / 180.0;
    }

    int Device::convert_rad_to_deg(double rad) {
        int deg = static_cast<int>(std::lround(rad * 180.0 / M_PI)) % 360;
        return deg;
    }
    int Device::normalize_deg(int deg) {
        deg = deg % 360;
        if (deg < -180) {
            deg += 360;
        } else if (deg >= 180) {
            deg -= 360;
        }
        return deg;
    }

    double Device::normalize_rad(double rad) {
        rad = std::fmod(rad, 2.0 * M_PI);
        if (rad < -M_PI) {
            rad += 2.0 * M_PI;
        } else if (rad >= M_PI) {
            rad -= 2.0 * M_PI;
        }
        return rad;
    }
} // namespace tuw_build_hat
