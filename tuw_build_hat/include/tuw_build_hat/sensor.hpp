#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>

namespace tuw_build_hat {

class Sensor {
  public:
    explicit Sensor(unsigned int id);
    virtual ~Sensor() = default;

    virtual void init() = 0;
    virtual void deactivate() = 0;

    void selrate(int ms);
    int selrate() const;

    void plimit(double power);
    double plimit() const;

    const std::string &cmd() const;

    std::string get_and_clear_command();

    bool has_cmd() const;


  protected:
    unsigned int id_;   // The senesors port id
    unsigned int selrate_; // defines how often firmware sends unsolicited sensor/actuator updates for the currently selected mode on that port [ms]
    double plimit_;
    std::string cmd_;
};

} // namespace tuw_build_hat

#endif
