#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visual_srv_msg/srv/obj_dir.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 10) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "usage: spawn obj visual file in rviz");
    return 1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n operation :%s \n location:%s",
              argv[1], argv[2]);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("visual_client");

  rclcpp::Client<visual_srv_msg::srv::ObjDir>::SharedPtr client =
      node->create_client<visual_srv_msg::srv::ObjDir>("visual_server");

  auto request = std::make_shared<visual_srv_msg::srv::ObjDir::Request>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client created");
  // strcpy((char *)request->operation.c_str(), argv[1]);
  // strcpy((char *)request->location.c_str(), argv[2]);
  std::string op(argv[1]);
  std::string loc(argv[2]);
  std::string object_id(argv[3]);
  std::string planning_scene_ns(argv[4]);
  std::string pose_x(argv[5]);
  std::string pose_y(argv[6]);
  std::string pose_z(argv[7]);
  std::string pose_w(argv[8]);
  std::string base_link(argv[9]);
  request->operation = op;
  request->location = loc;
  request->object_id = object_id;
  request->plan_scene_ns = planning_scene_ns;
  request->pose_x = pose_x;
  request->pose_y = pose_y;
  request->pose_z = pose_z;
  request->pose_w = pose_w;
  request->base_link = base_link;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "operation recived and location");
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    std::string co_id(result.get()->id);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Collision object id is: %s ",
                co_id.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ");
  }

  rclcpp::shutdown();
  return 0;
}