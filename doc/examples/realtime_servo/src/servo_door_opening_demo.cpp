/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : servo_door_opening_demo.cpp
 *      Project   : moveit_servo
 *      Created   : 07/20/2023
 *      Author    : V Mohammed Ibrahim
 *      Description : Example of controlling a robot through pose commands via the C++ API.
 */

#include <atomic>
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

using namespace moveit_servo;

// Utility for converting pose in any frame to pose in planning frame.
// Assuming a transform is available through the tf library.
PoseCommand toPlanningFrame(const PoseCommand& pose_command, tf2_ros::Buffer& transform_buffer,
                            const std::string& planning_frame)
{
  auto pose_command_msg = convertIsometryToTransform(pose_command.pose, planning_frame, pose_command.frame_id);
  auto command_to_planning_frame = transform_buffer.lookupTransform(planning_frame, pose_command.frame_id,
                                                                    rclcpp::Time(0), rclcpp::Duration::from_seconds(2));

  tf2::doTransform(pose_command_msg, pose_command_msg, command_to_planning_frame);
  return PoseCommand{ planning_frame, tf2::transformToEigen(pose_command_msg) };
}

// Utility for converting twist in any frame to twist in planning frame.
// Assuming a transform is available through the tf library.
TwistCommand toPlanningFrame(const TwistCommand& twist_command, tf2_ros::Buffer& transform_buffer,
                             const std::string& planning_frame)
{
  auto command_to_planning_frame = transform_buffer.lookupTransform(planning_frame, twist_command.frame_id,
                                                                    rclcpp::Time(0), rclcpp::Duration::from_seconds(2));
  Eigen::VectorXd transformed_twist = twist_command.velocities;
  tf2::doTransform(transformed_twist, transformed_twist, command_to_planning_frame);
  return TwistCommand{ planning_frame, transformed_twist };
}

void moveToPose(Servo& servo, const PoseCommand& target_pose,
                rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& trajectory_pub)
{
  bool pose_achieved = false;
  KinematicState joint_state;
  servo::Params servo_params = servo.getParams();

  rclcpp::WallRate tracking_rate(50);
  while (rclcpp::ok() && !pose_achieved)
  {
    pose_achieved = servo.getEndEffectorPose().isApprox(target_pose.pose, servo_params.pose_tracking.linear_tolerance);

    joint_state = servo.getNextJointState(target_pose);
    if (servo.getStatus() != StatusCode::INVALID)
      trajectory_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
    tracking_rate.sleep();
  }
}

void applyTwist(Servo& servo, const TwistCommand& target_twist,
                rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& trajectory_pub)
{
  servo::Params servo_params = servo.getParams();

  // Frequency at which the commands will be send to robot controller.
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  std::chrono::seconds timeout_duration(5);
  std::chrono::seconds time_elapsed(0);
  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok() && time_elapsed < timeout_duration)
  {
    KinematicState joint_state = servo.getNextJointState(target_twist);

    auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

    if (servo.getStatus() != StatusCode::INVALID)
    {
      trajectory_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
    }

    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  auto tutorial_node = std::make_shared<rclcpp::Node>("servo_tutorial");
  tf2_ros::Buffer transform_buffer(tutorial_node->get_clock());
  tf2_ros::TransformListener transform_listener(transform_buffer);

  // Get the servo parameters.
  std::string param_namespace = "moveit_servo";
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(tutorial_node, param_namespace);
  auto servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub =
      tutorial_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                             rclcpp::SystemDefaultsQoS());

  // Create the servo object
  auto planning_scene_monitor = createPlanningSceneMonitor(tutorial_node, servo_params);
  auto servo = Servo(tutorial_node, servo_param_listener, planning_scene_monitor);

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // For syncing pose tracking thread and main thread.
  std::mutex pose_guard;
  std::atomic<bool> stop_tracking = false;

  // Set the command type for servo.
  servo.expectedCommandType(CommandType::POSE);

  // Define the pose of the door knob to be  +15 cm in the current ee frame
  PoseCommand door_knob_pose;
  door_knob_pose.frame_id = servo_params.planning_frame;
  door_knob_pose.pose = servo.getEndEffectorPose();  // Eigen::Isometry3d::Identity();
  door_knob_pose.pose.translate(Eigen::Vector3d(0.1, 0.0, 0.0));

  // door_knob_pose = toPlanningFrame(door_knob_pose, transform_buffer, servo_params.planning_frame);

  moveToPose(servo, door_knob_pose, trajectory_pub);

  servo.expectedCommandType(CommandType::TWIST);

  TwistCommand target_twist{ servo_params.ee_frame, { 0.0, 0.0, 0.1, 0.0, 0.0, 0.5 } };
  applyTwist(servo, toPlanningFrame(target_twist, transform_buffer, servo_params.planning_frame), trajectory_pub);

  return 0;
}
