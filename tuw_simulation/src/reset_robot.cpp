#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ignition/transport/Node.hh"
#include "ignition/msgs/world_control.pb.h"
#include "ignition/msgs/boolean.pb.h"
#include "ignition/msgs/pose.pb.h"
#include "ignition/msgs/model.pb.h"
#include "ignition/msgs/entity.pb.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("minimal_client");

    auto x = 0.0;
    auto y = 0.0;
    double z = 0.4;
    std::string name = "robot0";

    gz::transport::Node node_ign;
    bool result;

    // ignition::msgs::Pose pose_req;
    // ignition::msgs::Boolean pose_res;

    // pose_req.set_name(name);
    // pose_req.mutable_position()->set_x(x);
    // pose_req.mutable_position()->set_y(y);
    // pose_req.mutable_position()->set_z(z);

    // bool executed =
    // node_ign.Request("/world/plain_world/set_pose", pose_req, 1000, pose_res, result);
    // if (executed) {
    //     if (result)
    //         std::cerr << "Entity pose was set : [" << pose_res.data() << "]"
    //                 << std::endl;
    //     else {
    //         std::cerr << "Service call failed" << std::endl;
    //     }
    // } else {
    //         std::cerr << "Service call timed out" << std::endl;
    // }

    // ignition::msgs::WorldControl reset_req;
    // ignition::msgs::WorldControl test;
    // ignition::msgs::Boolean reset_res;

    // reset_req.mutable_reset()->set_all(true);

    // executed =
    // node_ign.Request("/world/plain_world/control", reset_req, 3000, reset_res, result);
    // if (executed) {
    //     if (result)
    //         std::cerr << "Time was reseted : [" << reset_res.data() << "]"
    //                 << std::endl;
    //     else {
    //         std::cerr << "Service call failed" << std::endl;
    //     }
    // } else {
    //         std::cerr << "Service call timed out" << std::endl;
    // }
    

    rclcpp::shutdown();

    return 0;
}