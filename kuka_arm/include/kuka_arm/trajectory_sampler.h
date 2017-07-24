/*******************************************************************************
 * Copyright (C) 2017 Electric Movement Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#ifndef KUKA_ARM_TRAJECTORY_SAMPLER_H
#define KUKA_ARM_TRAJECTORY_SAMPLER_H

#include <string>
#include <sstream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shape_operations.h>

#include <eigen_conversions/eigen_msg.h>

#include <kuka_arm/CalculateIK.h>

class TrajectorySampler
{
public:
  explicit TrajectorySampler(ros::NodeHandle nh);
  ~TrajectorySampler();

private:
  ros::NodeHandle nh_;

  int cycle_counter;
  std::string target_description_param;

  const std::string PLANNING_GROUP = "arm_group";
  const std::string GRIPPER_GROUP = "gripper_group";
  const std::string SHELF_MESH_PATH =
    "package://kuka_arm/models/kinematics_shelf/kinematics_shelf.dae";
  const std::string BIN_MESH_PATH =
    "package://kuka_arm/models/kinematics_bin/kinematics_bin.dae";


  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::MoveGroupInterface eef_group;

  const robot_state::JointModelGroup *joint_model_group;
  const robot_state::JointModelGroup *gripper_joint_model_group;

  // Define PlanningSceneInterface object to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /*
   * Functions for gripper actuation
   * close_gripper = 0; open gripper
   *                 = 1; close gripper
   */
  bool OperateGripper(const bool &close_gripper);
  bool OpenGripper();
  bool CloseGripper();

  bool SetupCollisionObject(const std::string &object_id,
                            const std::string &mesh_path,
                            const geometry_msgs::Pose &object_pose,
                            moveit_msgs::CollisionObject &collision_object);
};

#endif  // KUKA_ARM_TRAJECTORY_SAMPLER_H
