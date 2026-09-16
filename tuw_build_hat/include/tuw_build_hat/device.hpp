#ifndef TUW_BUILD_HAT__DEVICE_HPP
#define TUW_BUILD_HAT__DEVICE_HPP

#include <string>

namespace tuw_build_hat {

class Device {
  public:
    explicit Device(unsigned int port);
    virtual ~Device() = default;

    virtual const std::string &info() const = 0;
    virtual void init() = 0;
    virtual void deactivate() = 0;
    virtual int decode(const std::string &msg) = 0;
    virtual bool is_feedback(const std::string &msg) const = 0;

    unsigned int port();

    void selrate(int ms);
    int selrate() const;

    void plimit(double power);
    double plimit() const;

    const std::string &cmd() const;

    std::string get_and_clear_command();

    bool has_cmd() const;

    static int convert_speed_to_percent(double rad_per_sec, double max_rad_per_sec);
    static double convert_percent_to_speed(int percent, double max_rad_per_sec);
    static double convert_deg_to_rad(double deg);
    static int convert_rad_to_deg(double rad);
    static int normalize_deg(int deg);
    static double normalize_rad(double rad);

  protected:
    unsigned int port_;    // The device port id
    unsigned int selrate_; // defines how often firmware sends unsolicited sensor/actuator updates for the currently selected mode on that port [ms]
    double plimit_;
    std::string cmd_;
};

} // namespace tuw_build_hat

#endif // TUW_BUILD_HAT__DEVICE_HPP
