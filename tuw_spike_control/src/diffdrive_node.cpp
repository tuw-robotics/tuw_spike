#include "tuw_spike_control/diffdrive_node.hpp"

#include <memory>

#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace tuw_spike_control {

DiffDriveNode::DiffDriveNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("diffdrive", options)
{
  wheel_separation_ = this->declare_parameter("wheel_separation", wheel_separation_);
  wheel_radius_ = this->declare_parameter("wheel_radius", wheel_radius_);

  hardware_ready_ = init_hardware_interface();
  if (!hardware_ready_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "hardware interface not available - cmd_vel will be converted but not actuated");
  }

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&DiffDriveNode::on_cmd_vel, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "diffdrive listening on '%s' (geometry_msgs/TwistStamped), "
    "wheel_separation=%.3f m, wheel_radius=%.3f m",
    cmd_vel_sub_->get_topic_name(), wheel_separation_, wheel_radius_);
}

DiffDriveNode::~DiffDriveNode()
{
  // The TuwSpikeSystemInterface destructor runs on_deactivate()/on_cleanup(),
  // which stops both motors and closes the serial port.
}

bool DiffDriveNode::init_hardware_interface()
{
  // Minimal HardwareInfo with the two wheel joints the interface expects.
  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info.name = "tuw_spike";
  init_params.hardware_info.type = "system";
  init_params.hardware_info.rw_rate = 100;

  hardware_interface::ComponentInfo left_wheel;
  left_wheel.name = "left_wheel_joint";
  left_wheel.type = "joint";
  hardware_interface::ComponentInfo right_wheel;
  right_wheel.name = "right_wheel_joint";
  right_wheel.type = "joint";
  init_params.hardware_info.joints = {left_wheel, right_wheel};

  if (lifecycle_.on_init(init_params) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "hardware interface on_init() failed");
    return false;
  }

  const rclcpp_lifecycle::State previous_state;
  if (lifecycle_.on_configure(previous_state) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "hardware interface on_configure() failed");
    return false;
  }
  if (lifecycle_.on_activate(previous_state) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "hardware interface on_activate() failed");
    return false;
  }

  // Grab the exported command interfaces and pick the two wheel velocity ones.
  // (export_command_interfaces() is the legacy hook this hardware still implements.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  command_interfaces_ = lifecycle_.export_command_interfaces();
#pragma GCC diagnostic pop
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() != hardware_interface::HW_IF_VELOCITY) {
      continue;
    }
    if (command_interface.get_prefix_name() == "left_wheel_joint") {
      left_wheel_velocity_ = &command_interface;
    } else if (command_interface.get_prefix_name() == "right_wheel_joint") {
      right_wheel_velocity_ = &command_interface;
    }
  }
  if (left_wheel_velocity_ == nullptr || right_wheel_velocity_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "wheel velocity command interfaces not found");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "hardware interface active");
  return true;
}

void DiffDriveNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped &msg)
{
  const double v = msg.twist.linear.x;    // [m/s] body forward velocity
  const double w = msg.twist.angular.z;   // [rad/s] body yaw rate

  // Differential-drive inverse kinematics.
  const double v_left = v - w * wheel_separation_ / 2.0;
  const double v_right = v + w * wheel_separation_ / 2.0;

  const double omega_left =  v_left / wheel_radius_;
  const double omega_right = v_right / wheel_radius_;

  RCLCPP_INFO(
    this->get_logger(),
    "cmd_vel [stamp %u.%09u frame '%s']: v=%.3f m/s w=%.3f rad/s -> "
    "wheel omega L=%.3f rad/s R=%.3f rad/s",
    msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id.c_str(),
    v, w, omega_left, omega_right);

  if (!hardware_ready_) {
    return;
  }

  // set_value() only updates the command buffer; write() pushes it to the motors.
  (void)left_wheel_velocity_->set_value(-omega_left);
  (void)right_wheel_velocity_->set_value(omega_right);

  const rclcpp::Time time = this->now();
  static rclcpp::Time last_time = time;
  const rclcpp::Duration period =
    (time > last_time) ? (time - last_time) : rclcpp::Duration::from_seconds(0.02);
  last_time = time;
  lifecycle_.write(time, period);
}

}  // namespace tuw_spike_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tuw_spike_control::DiffDriveNode>());
  rclcpp::shutdown();
  return 0;
}
