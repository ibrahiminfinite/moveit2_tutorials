

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

class Door
{
public:
  Door(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    dims_ = Eigen::Vector3d(0.5, 0.02, 0.8);
    rotation_radius_ = dims_.x() / 2;
    // Hinge is the bottom left corner
    hinge_ = Eigen::Vector3d(0.8, 0.0, 0.0);

    center_.x() = hinge_.x() + dims_.x() / 2;
    center_.y() = hinge_.y() + dims_.y() / 2 + rotation_radius_;
    center_.z() = hinge_.z() + dims_.z() / 2;

    angle_ = (M_PI / 2);

    collision_object_publisher_ =
        node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    createCollisionObject();
  }

  void rotateDoor(double angle)
  {
    angle_ = angle;

    collision_object_.primitives[0] = door_primitive_;

    center_.x() = hinge_.x() + (rotation_radius_ * cos(angle_));
    center_.y() = hinge_.y() + (rotation_radius_ * sin(angle_));
    geometry_msgs::msg::Pose door_pose;
    door_pose.position.x = center_.x();
    door_pose.position.y = center_.y();
    door_pose.position.z = center_.z();
    auto orn = Eigen::Quaterniond(Eigen::AngleAxisd(angle_, Eigen::Vector3d::UnitZ()));
    door_pose.orientation.w = orn.w();
    door_pose.orientation.x = orn.x();
    door_pose.orientation.y = orn.y();
    door_pose.orientation.z = orn.z();

    collision_object_.operation = collision_object_.ADD;
    collision_object_.primitive_poses[0] = door_pose;
    collision_object_.header.stamp = node_->now();

    moveit_msgs::msg::PlanningSceneWorld psw;
    psw.collision_objects.push_back(collision_object_);

    moveit_msgs::msg::PlanningScene ps;
    ps.world = psw;
    ps.is_diff = true;
    collision_object_publisher_->publish(ps);
  }

private:
  void createCollisionObject()
  {
    collision_object_.id = "door";
    collision_object_.header.frame_id = "panda_link0";
    collision_object_.primitives.resize(1);
    collision_object_.primitive_poses.resize(1);

    door_primitive_.type = shape_msgs::msg::SolidPrimitive::BOX;
    door_primitive_.dimensions = { dims_[0], dims_[1], dims_[2] };
  }

  rclcpp::Node::SharedPtr node_;
  Eigen::Vector3d hinge_, center_, dims_;
  double angle_, step_, rotation_radius_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_object_publisher_;
  moveit_msgs::msg::CollisionObject collision_object_;
  shape_msgs::msg::SolidPrimitive door_primitive_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("servo_tutorial");
  auto marker_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", rclcpp::SystemDefaultsQoS());
  auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/servo_node/pose_target_cmds",
                                                                                rclcpp::SystemDefaultsQoS());

  int id = 0;
  auto get_marker = [&](const Eigen::Vector3d& position, const std::string& frame) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Time(0.0);
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    id++;
    return marker;
  };

  double start_angle = M_PI / 2 + (M_PI / 8);
  double end_angle = M_PI;
  double step = 0.01745329;

  visualization_msgs::msg::MarkerArray marray;
  std::vector<Eigen::Vector3d> traj;
  std::vector<double> door_angles;

  for (double i = start_angle; i < end_angle; i = i + step)
  {
    double x = 0.8 + (0.5 * cos(i));
    double y = 0.0 + (0.5 * sin(i));
    auto vec = Eigen::Vector3d(x, y, 0.4);
    traj.push_back(vec);
    door_angles.push_back(i);
    marray.markers.push_back(get_marker(vec, "panda_link0"));
  }
  marker_publisher->publish(marray);

  Door door(node);
  door.rotateDoor(M_PI / 2);

  auto ee_pose = Eigen::Isometry3d::Identity();
  ee_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  ee_pose.translate(Eigen::Vector3d(0.3, 0.0, 0.4));

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "panda_link0";
  auto rotation = Eigen::Quaterniond(ee_pose.rotation());
  target_pose.pose.orientation.w = rotation.w();
  target_pose.pose.orientation.x = rotation.x();
  target_pose.pose.orientation.y = rotation.y();
  target_pose.pose.orientation.z = rotation.z();
  target_pose.pose.position.z = 0.4;

  size_t i = traj.size() - 1;
  while (i > 0)
  {
    target_pose.pose.position.x = traj[i].x();
    target_pose.pose.position.y = traj[i].y();
    target_pose.header.stamp = node->now();
    pose_publisher->publish(target_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    i--;
  }

  double door_angle = M_PI / 2;
  i = 0;
  while (i < traj.size())
  {
    ee_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    target_pose.pose.position.x = traj[i].x();
    target_pose.pose.position.y = traj[i].y();
    target_pose.header.stamp = node->now();
    pose_publisher->publish(target_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    door.rotateDoor(door_angle);
    door_angle += step;
    i++;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
}
