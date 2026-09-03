#ifndef TUW_SPIKE_CONTROL__DIFFDRIVE_NODE_HPP_
#define TUW_SPIKE_CONTROL__DIFFDRIVE_NODE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tuw_spike_control/tuw_spike_control_interface.hpp"

namespace tuw_spike_control {

/// @brief Minimal differential-drive node.
/// @details
/// Subscribes to a stamped twist command on `cmd_vel`
/// (geometry_msgs/msg/TwistStamped), converts the body twist
/// (linear.x, angular.z) into left/right wheel angular velocities using a
/// differential-drive kinematic model, and pushes them to the
/// tuw_spike_control hardware interface which is brought up in the constructor.
class DiffDriveNode : public rclcpp::Node {
  public:
    explicit DiffDriveNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DiffDriveNode() override;

  private:
    /// @brief Run on_init()/on_configure()/on_activate() on the hardware
    ///        interface and cache the wheel velocity command handles.
    /// @return true when the interface is active and both wheels were found.
    bool init_hardware_interface();

    void on_cmd_vel(const geometry_msgs::msg::TwistStamped &msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

    double wheel_separation_{0.112};   // [m] distance between the two wheels
    double wheel_radius_{0.028};      // [m] wheel radius

    // Hardware interface provided by the tuw_spike_control_interface library.
    TuwSpikeSystemInterface interface_;
    hardware_interface::SystemInterface &lifecycle_{interface_};
    bool hardware_ready_{false};

    std::vector<hardware_interface::CommandInterface> command_interfaces_;
    hardware_interface::CommandInterface *left_wheel_velocity_{nullptr};
    hardware_interface::CommandInterface *right_wheel_velocity_{nullptr};
};

}  // namespace tuw_spike_control

#endif  // TUW_SPIKE_CONTROL__DIFFDRIVE_NODE_HPP_
