#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ignition/msgs/entity_factory.pb.h"
#include "ignition/transport/Node.hh"
#include "ignition/msgs/stringmsg.pb.h"
#include "ignition/msgs/scene.pb.h"
#include "ignition/msgs/stringmsg_v.pb.h"
#include "ignition/msgs/world_control.pb.h"
#include "ignition/msgs/entity.pb.h"

using namespace std::chrono_literals;

static std::optional<std::string> robot_description = std::nullopt;

void read_description(const std_msgs::msg::String & msg)
{
    robot_description = msg.data;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("robot_spawner");
    auto sub = node->create_subscription<std_msgs::msg::String>("robot_description", 10, read_description);
    
    while (!robot_description) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(10ms);
    }
   
    auto x = node->declare_parameter<double>("X", 0.0);
    auto y = node->declare_parameter<double>("Y", 0.0);
    double z = 0.1;

    std::string name = node->get_effective_namespace();
    name = name.substr(1);
    if (name.empty())
    {
        name = "robot0";
    }
    
    bool exists = false;
    bool success = true;

    ignition::msgs::Entity test;

    gz::transport::Node node_ign;
    ignition::msgs::Scene res;
    ignition::msgs::Empty req;

    bool result;
    bool r = node_ign.Request("/world/plain_world/scene/info", req, 20000, res, result);
    if (r) {
        if (result) {
            for (int i = 0; i < res.model_size(); i++) {
                auto tmp = res.model(i).name();
                //RCLCPP_INFO(node->get_logger(), "%s", tmp.c_str());
                if (name.compare(tmp) == 0) {
                    RCLCPP_INFO(node->get_logger(), "model already exists");
                    exists = true;
                    success = false;
                }
            }
        } else {
            std::cerr << "scene info service call failed" << std::endl;
            success = false;
        }
    } else {
        std::cerr << "scene info service call timed out" << std::endl;
        success = false;
    }
    
    if (!exists && success) {
        ignition::msgs::EntityFactory req_c;
        ignition::msgs::Boolean res_c;
        req_c.set_sdf(*robot_description);
        req_c.set_name(name);
        req_c.mutable_pose()->mutable_position()->set_x(x);
        req_c.mutable_pose()->mutable_position()->set_y(y);
        req_c.mutable_pose()->mutable_position()->set_z(z);
        bool executed =
            node_ign.Request("/world/plain_world/create", req_c, 20000, res_c, result);
        if (executed) {
            if (result) {
                std::cerr << "Entity was created : [" << res_c.data() << "]"
                        << std::endl;
                success = true;
            } else {
                std::cerr << "create service call failed" << std::endl;
                success = false;
            }
        } else {
                std::cerr << "create service call timed out" << std::endl;
                success = false;
            }
    }
    rclcpp::shutdown();

    return success ? 0 : 1;
}

