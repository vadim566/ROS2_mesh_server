#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <igl/readOBJ.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>

class myMashobj {
private:
public:
  int resize = 1000;
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  shape_msgs::msg::Mesh mesh;
  moveit_msgs::msg::CollisionObject co;
  geometry_msgs::msg::Pose pose;
  std::string base_link;
  float pose_x, pose_y, pose_z, pose_w;
  myMashobj(std::string mesh_loc, std::string object_id,
            moveit::planning_interface::PlanningSceneInterface
                planning_scene_interface,
            float pose_x, float pose_y, float pose_z, float pose_w,
            std::string base_link);
  void transformObjVertics();
  void transformObjTriangles();
  void setPose();
  ~myMashobj() {}
};

void myMashobj::setPose() {
  this->pose.position.x = this->pose_x;
  this->pose.position.y = this->pose_y;
  this->pose.position.z = this->pose_z;
  this->pose.orientation.w = this->pose_w;
  this->pose.orientation.x = 0;
  this->pose.orientation.y = 0;
  this->pose.orientation.z = 0;
  this->co.mesh_poses.push_back(this->pose);
}
void myMashobj::transformObjVertics() {
  this->mesh.vertices.resize(this->V.rows());
  for (int i = 0; i < this->V.rows(); ++i) {
    this->mesh.vertices[i].x = V(i, 0) / this->resize;
    this->mesh.vertices[i].y = V(i, 1) / this->resize;
    this->mesh.vertices[i].z = V(i, 2) / this->resize;
  }
}
void myMashobj::transformObjTriangles() {
  this->mesh.triangles.resize(this->F.rows());
  for (int i = 0; i < this->F.rows(); ++i) {
    this->mesh.triangles[i].vertex_indices[0] = this->F(i, 0);
    this->mesh.triangles[i].vertex_indices[1] = this->F(i, 1);
    this->mesh.triangles[i].vertex_indices[2] = this->F(i, 2);
  }
}

myMashobj::myMashobj(
    std::string mesh_loc, std::string object_id,
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface,
    float pose_x, float pose_y, float pose_z, float pose_w,
    std::string base_link)
    : pose_x(pose_x), pose_y(pose_y), pose_z(pose_z), pose_w(pose_w),
      base_link(base_link) {
  // read obj file from location
  igl::readOBJ(mesh_loc, this->V, this->F);

  // transfrom obj to vertics and traingles
  transformObjVertics();

  transformObjTriangles();

  // Create the moveit_msgs::CollisionObject message

  this->co.header.frame_id = this->base_link;
  this->co.id = object_id;
  this->co.meshes.push_back(this->mesh);

  this->co.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Set the pose of the Collision Object
  setPose();
  // Add the Collision Object to the planning scene
  planning_scene_interface.addCollisionObjects(
      std::vector<moveit_msgs::msg::CollisionObject>(1, co));
  planning_scene_interface.applyCollisionObjects(
      std::vector<moveit_msgs::msg::CollisionObject>(1, co));
}
