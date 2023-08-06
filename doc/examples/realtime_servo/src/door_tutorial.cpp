

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("servo_tutorial");

    auto collision_object_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene",
        rclcpp::SystemDefaultsQoS());

    auto marker_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_markers",
        rclcpp::SystemDefaultsQoS()
    );

    auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/servo_node/pose_target_cmds",
        rclcpp::SystemDefaultsQoS()
    );

    auto switch_input = node->create_client<moveit_msgs::srv::ServoCommandType>("servo_node/switch_command_type");


    double start_angle = 3.14;
    double end_angle = start_angle + (3.14/3);
    double step = 0.01745329;

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_link0";
    collision_object.id = "door";

    shape_msgs::msg::SolidPrimitive door;
    door.type = door.BOX;
    door.dimensions = { 0.02, 0.5, 0.8 };

    int id = 0;
    auto get_marker = [&](const Eigen::Vector3d& position, const std::string& frame)
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        id++;
        return marker;
    };
    
    visualization_msgs::msg::MarkerArray marray;
    std::vector<Eigen::Vector3d> traj;
    std::vector<double> door_angles;

    for(double i=start_angle; i < end_angle; i = i + step)
    {
        double x = 0.8 + (0.5 * cos(i));
        double y = 0.0 + (0.5 * sin(i));
        auto vec = Eigen::Vector3d(x, y, 0.6);
        traj.push_back(vec);
        door_angles.push_back(i);
        marray.markers.push_back(get_marker(vec, "panda_link0"));
    }
    marker_publisher->publish(marray);


    geometry_msgs::msg::Pose door_pose;
    door_pose.position.x = 0.8 + (0.25 * cos(door_angles[traj.size() - 1]));
    door_pose.position.y = (0.25 * sin(door_angles[traj.size() - 1]));
    door_pose.position.z = 0.4;
    auto orn = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2 + end_angle, Eigen::Vector3d::UnitZ()));
    door_pose.orientation.w = orn.w();
    door_pose.orientation.x = orn.x();
    door_pose.orientation.y = orn.y();
    door_pose.orientation.z = orn.z();
    collision_object.primitives.push_back(door);
    collision_object.primitive_poses.push_back(door_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::msg::PlanningSceneWorld psw;
    psw.collision_objects.push_back(collision_object);

    moveit_msgs::msg::PlanningScene ps;
    ps.world = psw;
    ps.is_diff = true;
    collision_object_publisher->publish(ps);


    auto ee_pose = Eigen::Isometry3d::Identity();
    ee_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    ee_pose.translate(Eigen::Vector3d(0.3, 0.0, 0.6));


    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    auto rotation = Eigen::Quaterniond(ee_pose.rotation());
    target_pose.pose.orientation.w = rotation.w();
    target_pose.pose.orientation.x = rotation.x();
    target_pose.pose.orientation.y = rotation.y();
    target_pose.pose.orientation.z = rotation.z();
    target_pose.pose.position.z = 0.6;

    size_t i = 0;
    while(i < traj.size())
    {
        target_pose.pose.position.x = traj[i].x();
        target_pose.pose.position.y = traj[i].y();
        target_pose.header.stamp = node->now();
        pose_publisher->publish(target_pose);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        i++;
    }

    i = traj.size() - 1;
    while(i > 0)
    {
        target_pose.pose.position.x = traj[i].x();
        target_pose.pose.position.y = traj[i].y();
        target_pose.header.stamp = node->now();
        pose_publisher->publish(target_pose);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        geometry_msgs::msg::Pose door_pose;
        door_pose.position.x = 0.8 + (0.25 * cos(door_angles[i]));
        door_pose.position.y = (0.25 * sin(door_angles[i]));
        door_pose.position.z = 0.4;
        auto orn = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2 + end_angle, Eigen::Vector3d::UnitZ()));
        door_pose.orientation.w = orn.w();
        door_pose.orientation.x = orn.x();
        door_pose.orientation.y = orn.y();
        door_pose.orientation.z = orn.z();
        psw.collision_objects[0].operation = collision_object.ADD;
        psw.collision_objects[0].primitive_poses[0] = door_pose;
        moveit_msgs::msg::PlanningScene ps;
        ps.world = psw;
        ps.is_diff = true;
        collision_object_publisher->publish(ps);
        end_angle -= step;
        
        i--;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
}