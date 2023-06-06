#include "myMashobj.h"
#include "rclcpp/rclcpp.hpp"
#include "visual_srv_msg/srv/obj_dir.hpp"
#include <functional>
#include <memory>
#include <rviz_visual_tools/remote_control.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>

// C++
#include <string>
#include <vector>

// Msgs
#include <geometry_msgs/msg/vector3.hpp>
namespace scene_plane {}

class objVisualServer {
public:
  objVisualServer();
  static void
  load_obj(const std::shared_ptr<visual_srv_msg::srv::ObjDir::Request> request,
           std::shared_ptr<visual_srv_msg::srv::ObjDir::Response> response);

private:
};

objVisualServer::objVisualServer() {}

void objVisualServer::load_obj(
    const std::shared_ptr<visual_srv_msg::srv::ObjDir::Request> request,
    std::shared_ptr<visual_srv_msg::srv::ObjDir::Response> response) {

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recieved request");

  // init items from request
  std::string co_op(request->operation);
  std::string loc(request->location);
  std::string object_id(request->object_id);
  std::string scene_ns(request->plan_scene_ns);
  float pose_x = std::stof(request->pose_x);
  float pose_y = std::stof(request->pose_y);
  float pose_z = std::stof(request->pose_z);
  float pose_w = std::stof(request->pose_w);
  std::string base_link(request->base_link);

  // end of init items from request

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recognized operation request");

  // create general planning scene and collision object
  moveit::planning_interface::PlanningSceneInterface *planning_si;
  planning_si =
      new moveit::planning_interface::PlanningSceneInterface(scene_ns);
  myMashobj *co;

  // operators
  if (!strcmp(co_op.c_str(), "add")) {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recognized add  request");

    co = new myMashobj(loc, object_id, *planning_si, pose_x, pose_y, pose_z,
                       pose_w, base_link);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "created collision object id %s",
                co->co.id.c_str());

    response->id = co->co.id;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "respond created:%s",
                co->co.id.c_str());

  }

  else if (!strcmp(co_op.c_str(), "remove")) {
    std::vector<std::string> object_ids;

    object_ids.push_back(object_id);

    planning_si->removeCollisionObjects(object_ids);
    // myMashobj co = myMashobj(request->location);
    response->id = "removed";

  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no matching operator");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "finish handling  operation request");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("visual_server_node");

  std::shared_ptr<objVisualServer> visual_server =
      std::make_shared<objVisualServer>();

  rclcpp::Service<visual_srv_msg::srv::ObjDir>::SharedPtr service =
      node->create_service<visual_srv_msg::srv::ObjDir>(
          "visual_server", &(visual_server->load_obj));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "waiting to requests to visualise obj file");

  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "something happend");

  //   executor->remove_node(demo);

  //   RCLCPP_INFO(rclcpp::get_logger("rviz_demo"), "Shutting down.");

  rclcpp::shutdown();
}
