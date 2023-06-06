cmake_minimum_required(VERSION 3.8)
project(visual_obj_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visual_srv_msg REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()




find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)

find_package(rviz_visual_tools)


set(THIS_PACKAGE_INCLUDE_DEPENDS

  

)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(server src/visual_server.cpp)
ament_target_dependencies(server
rclcpp visual_srv_msg moveit_ros_planning_interface rviz_visual_tools)


add_executable(client src/visual_client.cpp)
ament_target_dependencies(client
rclcpp visual_srv_msg moveit_ros_planning_interface)

install(TARGETS
client
  server
  DESTINATION lib/${PROJECT_NAME})
ament_package()
