#include "tuw_build_hat/sensor.hpp"

#include <format>

namespace tuw_build_hat {

Sensor::Sensor(unsigned int id) : id_(id), selrate_(100) {}

    void Sensor::selrate(int ms){
        selrate_ = ms;
    }
    int Sensor::selrate() const{
        return selrate_;
    }

    void Sensor::plimit(double power){
        plimit_ = power;
    }
    double Sensor::plimit() const{
        return plimit_;
    }

    const std::string &Sensor::cmd() const{
      return cmd_;
    }

    std::string Sensor::get_and_clear_command(){
      std::string ret(cmd_);
      cmd_.clear();
      return ret;
    }

    bool Sensor::has_cmd() const{
      return !cmd_.empty();
    }
} // namespace tuw_build_hat
