/*******************************************************************************
 * Copyright (C) 2017 Electric Movement Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#include <kuka_arm/trajectory_sampler.h>

TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
  : nh_(nh),
    cycle_counter(0),
    move_group(PLANNING_GROUP),
    eef_group(GRIPPER_GROUP)
{
  /*
   * Setup:
   * Load robot model, robot state, set planning_scene
   * Define the move_group for planning and control purpose
   */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  robot_state::RobotState robot_kinematic_state(kinematic_model);

  // Get demo param
  bool demo;
  ros::param::get("trajectory_sampler/demo", demo);

  // set RRT as the planner and set allowed planning time
  move_group.setPlannerId("RRTkConfigDefault");
  move_group.setPlanningTime(10.0);
  eef_group.setPlannerId("RRTkConfigDefault");
  eef_group.setPlanningTime(5.0);

  // Pointer to JointModelGroup for improved performance.
  joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  gripper_joint_model_group =
    eef_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

  /*
   * Collision Objects:
   * Create an object list and populate it with shelf and bin objects
   * Then insert objects in scene for collision avoidance and interaction
   */
  std::vector<moveit_msgs::CollisionObject> collision_object_list;
  moveit_msgs::CollisionObject shelf_collision_object, bin_collision_object,
              ground_collision_object;

  ground_collision_object.header.frame_id = move_group.getPlanningFrame();
  ground_collision_object.id = "ground";

  shape_msgs::SolidPrimitive ground_prim;
  ground_prim.type = ground_prim.BOX;
  ground_prim.dimensions.resize(3);
  ground_prim.dimensions[0] = 6.0;
  ground_prim.dimensions[1] = 6.0;
  ground_prim.dimensions[2] = 0.05;

  // Define pose for the objects (specified relative to base_footprint)
  geometry_msgs::Pose bin_mesh_pose, shelf_mesh_pose, ground_prim_pose;
  ground_prim_pose.orientation.w = 1.0;
  ground_prim_pose.position.x = 0.0;
  ground_prim_pose.position.y = 0.0;
  ground_prim_pose.position.z = -0.025;

  shelf_mesh_pose.position.x = 2.7;
  shelf_mesh_pose.position.y = 0.0;
  shelf_mesh_pose.position.z = 0.84;
  shelf_mesh_pose.orientation.w = 0.707;
  shelf_mesh_pose.orientation.x = 0.0;
  shelf_mesh_pose.orientation.y = 0.0;
  shelf_mesh_pose.orientation.z = 0.707;

  bin_mesh_pose.position.x = 0.0;
  bin_mesh_pose.position.y = 2.5;
  bin_mesh_pose.position.z = 0.0;
  bin_mesh_pose.orientation.w = 1.0;
  bin_mesh_pose.orientation.x = 0.0;
  bin_mesh_pose.orientation.y = 0.0;
  bin_mesh_pose.orientation.z = 0.0;

  SetupCollisionObject("shelf", SHELF_MESH_PATH, shelf_mesh_pose,
                       shelf_collision_object);
  SetupCollisionObject("bin", BIN_MESH_PATH, bin_mesh_pose,
                       bin_collision_object);

  ground_collision_object.primitives.push_back(ground_prim);
  ground_collision_object.primitive_poses.push_back(ground_prim_pose);
  ground_collision_object.operation = ground_collision_object.ADD;

  collision_object_list.push_back(shelf_collision_object);
  collision_object_list.push_back(bin_collision_object);
  collision_object_list.push_back(ground_collision_object);

  // Add the object list to the world scene
  planning_scene_interface.addCollisionObjects(collision_object_list);
  ROS_INFO("Added object list to the world");

  // Allow MoveGroup to add the collision objects in the world
  ros::Duration(1.0).sleep();

  while (ros::ok())
  {
    /*
     * rviz visualization:
     * Setup MoveItVisualTools for visualizing collision objects, robot,
     * and trajectories in Rviz
     */
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Load RemoteControl for step-by-step progression
    visual_tools.loadRemoteControl();

    // Create text marker for displaying current state
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    Eigen::Affine3d instr_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 4.0;
    instr_pose.translation().z() = 3.5;
    visual_tools.publishText(text_pose, "Welcome to pick-place project!",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    // visual_tools.publishText(instr_pose, "Press 'Next' to continue...",
    // rviz_visual_tools::GREEN, rviz_visual_tools::XXXLARGE);

    // Publish messages to rviz
    visual_tools.trigger();
    visual_tools.prompt("next step");


    /*
     * Get Pick and Drop location from param server
     * Plan motion to pick location
     */
    float target_x, target_y, target_z;
    float bin_x, bin_y, bin_z;

    ros::param::get("/target_spawn_location/x", target_x);
    ros::param::get("/target_spawn_location/y", target_y);
    ros::param::get("/target_spawn_location/z", target_z);

    ros::param::get("/target_drop_location/x", bin_x);
    ros::param::get("/target_drop_location/y", bin_y);
    ros::param::get("/target_drop_location/z", bin_z);

    geometry_msgs::Pose target_pose, bin_pose, target_reach;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = target_x - 0.4;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z - 0.1;

    target_reach.orientation.w = 1.0;
    target_reach.position.x = target_x - 0.2;
    target_reach.position.y = target_y;
    target_reach.position.z = target_z - 0.1;

    bin_pose.orientation.w = 1.0;
    bin_pose.position.x = bin_x - 0.1;
    bin_pose.position.y = bin_y;
    bin_pose.position.z = bin_z + 1.6;

    // set starting pose
    move_group.setStartStateToCurrentState();

    // set target pose
    move_group.setPoseTarget(target_pose);

    // slow down movement of the robot
    move_group.setMaxVelocityScalingFactor(0.2);
    eef_group.setMaxVelocityScalingFactor(1.0);

    // define plan object which will hold the planned trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = move_group.plan(my_plan);
    ROS_INFO("Visualizing plan to target: %s",
             success ? "SUCCEEDED" : "FAILED");

    // Visualize the plan
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishText(text_pose, "Displaying plan to target location",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    /*
     * Convert plan to a set of eef poses for IK calculation
     */
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
      new robot_trajectory::RobotTrajectory(kinematic_model,
                                            joint_model_group->getName()));

    // RobotState contains the current position/velocity/acceleration data
    moveit::core::RobotStatePtr robot_current_state;
    std::vector<double> robot_joint_positions;

    // Vector to hold eef poses
    std::vector<geometry_msgs::Pose> path;
    std::size_t path_size;

    // Declare service client for IK
    ros::ServiceClient client =
      nh.serviceClient<kuka_arm::CalculateIK>("calculate_ik");
    kuka_arm::CalculateIK srv;

    /*
     * Execute the plan based on demo flag
     * if demo = 1; use moveit for IK
     *         = 0; use IK_server for IK
     */
    if (demo)
    {
      // Display current state
      visual_tools.publishText(text_pose, "Moving to the target location",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();
      // command the robot to execute the created plan
      success = move_group.execute(my_plan);
      ROS_INFO("Moving to pick location: %s",
               success ? "SUCCEEDED" : "FAILED");
    }

    else
    {
      robot_trajectory->setRobotTrajectoryMsg(robot_kinematic_state,
                                              my_plan.trajectory_);
      // Add eef poses from plan to our vector
      for (std::size_t i = 0; i < robot_trajectory->getWayPointCount(); ++i)
      {
        const Eigen::Affine3d& eef_pose =
          robot_trajectory->getWayPoint(i).getGlobalLinkTransform(joint_model_group->getLinkModel("link_6"));
        geometry_msgs::Pose gripper_pose;
        tf::poseEigenToMsg(eef_pose, gripper_pose);
        path.push_back(gripper_pose);
      }
      path_size = path.size();

      ROS_DEBUG_STREAM("Total poses in path: " << path.size()
                       << "Actual points in plan: "
                       << my_plan.trajectory_.joint_trajectory.points.size());
      ROS_DEBUG_STREAM("point1: " << path[0] << "point2: " << path[path_size - 1]);

      // Call service to calculate IK
      srv.request.poses = path;

      // Display current state
      visual_tools.publishText(text_pose, "Calculating Inverse Kinematics",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();

      if (client.call(srv))
      {
        ROS_DEBUG_STREAM("Current Pose: " << srv.response);
      }
      else
      {
        ROS_ERROR("Failed to call service calculate_ik");
      }

      /*
       * Extract joint angles from service response
       */
      // Get the current set of joint values for the group.
      robot_current_state = move_group.getCurrentState();
      robot_current_state->copyJointGroupPositions(joint_model_group,
          robot_joint_positions);

      ROS_DEBUG("Total joints in robot_joint_positions: %zd %zd",
                robot_joint_positions.size(), srv.response.points[0].positions.size());

      // Display current state
      visual_tools.publishText(text_pose, "Moving to the target location",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();

      // Use joints calculated by IK_server for motion
      for (std::size_t i = 0; i < srv.response.points.size(); ++i)
      {
        for (std::size_t j = 0; j < srv.response.points[0].positions.size(); ++j)
        {
          robot_joint_positions[j] = srv.response.points[i].positions[j];  // radians
        }

        move_group.setJointValueTarget(robot_joint_positions);
        bool worked = move_group.move();
        ROS_INFO("Robot actuation: %s", worked ? "SUCCEEDED" : "FAILED");
      }
    }

    // Display current state
    visual_tools.publishText(text_pose, "Reached target location",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    /*
     * Approach the target
     *
     */
    // Display current state
    visual_tools.publishText(text_pose, "Executing reaching movement",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_reach);
    success = move_group.move();
    ROS_INFO("Target reach: %s",
             success ? "SUCCEEDED" : "FAILED");

    visual_tools.prompt("next step");

    /*
     * Grasp the target
     */
    // Display current state
    visual_tools.publishText(text_pose, "Grasping target object",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();
    CloseGripper();
    visual_tools.prompt("next step");

    /*
     * Retract the gripper
     *
     */
    // Display current state
    visual_tools.publishText(text_pose, "Retrieving target object",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);
    success = move_group.move();
    ROS_INFO("Target retrieval: %s",
             success ? "SUCCEEDED" : "FAILED");

    visual_tools.prompt("next step");

    /*
     * Plan to bin location for drop-off
     */
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(bin_pose);

    success = move_group.plan(my_plan);
    ROS_INFO("Visualizing plan to drop location: %s",
             success ? "SUCCEEDED" : "FAILED");

    // Visualize the plan
    visual_tools.publishAxisLabeled(bin_pose, "drop_pose");
    visual_tools.publishText(text_pose, "Displaying plan to drop-off location",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    /*
     * Execute the plan based on demo flag
     * if demo = 1; use moveit for IK
     *         = 0; use IK_server for IK
     */
    if (demo)
    {
      // Display current state
      visual_tools.publishText(text_pose, "Moving to the drop-off location",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();
      // command the robot to execute the created plan
      success = move_group.execute(my_plan);
      ROS_INFO("Moving to drop location: %s",
               success ? "SUCCEEDED" : "FAILED");
    }

    /*
     * Convert plan to a set of eef poses for IK calculation
     */
    else
    {
      robot_trajectory->setRobotTrajectoryMsg(robot_kinematic_state, my_plan.trajectory_);

      // Clear the Vector that holds eef poses
      path.clear();

      // add eef poses from plan to our vector
      for (std::size_t i = 0; i < robot_trajectory->getWayPointCount(); ++i)
      {
        const Eigen::Affine3d& eef_pose =
          robot_trajectory->getWayPoint(i).getGlobalLinkTransform(joint_model_group->getLinkModel("link_6"));
        geometry_msgs::Pose gripper_pose;
        tf::poseEigenToMsg(eef_pose, gripper_pose);
        path.push_back(gripper_pose);
      }
      path_size = path.size();

      ROS_DEBUG_STREAM("Total poses in path: " << path.size()
                       << "Actual points in plan: "
                       << my_plan.trajectory_.joint_trajectory.points.size());
      ROS_DEBUG_STREAM("point1: " << path[0] << "point2: " << path[path_size - 1]);

      // Call service to calculate IK
      srv.request.poses = path;

      // Display current state
      visual_tools.publishText(text_pose, "Calculating Inverse Kinematics",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();

      if (client.call(srv))
      {
        ROS_INFO_STREAM("Response from Server: " << srv.response);
      }
      else
      {
        ROS_ERROR("Failed to call service calculate_ik");
      }

      /*
       * Extract joint angles from service response
       */

      // RobotState contains the current position/velocity/acceleration data
      robot_current_state = move_group.getCurrentState();

      // Next get the current set of joint values for the group.
      robot_joint_positions.clear();
      robot_current_state->copyJointGroupPositions(joint_model_group,
          robot_joint_positions);

      ROS_DEBUG("Total joints in robot_joint_positions: %zd %zd",
                robot_joint_positions.size(), srv.response.points[0].positions.size());

      // Display current state
      visual_tools.publishText(text_pose, "Moving to the drop-off location",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();

      // Use joints calculated by IK_server for motion
      for (std::size_t i = 0; i < srv.response.points.size(); ++i)
      {
        for (std::size_t j = 0; j < srv.response.points[0].positions.size(); ++j)
        {
          robot_joint_positions[j] = srv.response.points[i].positions[j];  // radians
        }

        move_group.setJointValueTarget(robot_joint_positions);
        bool worked = move_group.move();
        ROS_INFO("Robot actuation: %s", worked ? "SUCCEEDED" : "FAILED");
      }
    }

    ROS_INFO("Moving to drop location: %s",
             success ? "SUCCEEDED" : "FAILED");

    // Display current state
    visual_tools.publishText(text_pose, "Reached drop-off location",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Open the gripper and release the object
    // Display current state
    visual_tools.publishText(text_pose, "Releasing target object",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();
    OpenGripper();
    ros::Duration(2.0).sleep();

    // Update cycle cycle_counter
    cycle_counter++;

    // Display current state
    visual_tools.publishText(text_pose, "  End of Pick-Place cycle\nPress Next to begin a new cycle",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Move robot back to idle pose
    robot_current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    robot_joint_positions.clear();
    robot_current_state->copyJointGroupPositions(joint_model_group,
        robot_joint_positions);

    for (std::size_t j = 0; j < robot_joint_positions.size(); ++j)
    {
      robot_joint_positions[j] = 0;  // radians
    }

    move_group.setJointValueTarget(robot_joint_positions);
    success = move_group.move();
    ROS_INFO("Robot motion to Idle state: %s", success ? "SUCCEEDED" : "FAILED");

    // Spawn another target
    system("rosrun kuka_arm target_spawn.py");
    ros::Duration(1.0).sleep();

    float spawn_x, spawn_y, spawn_z;
    std::stringstream td_sstream;
    nh.getParam("/target_description_argument", target_description_param);

    td_sstream << "rosrun gazebo_ros spawn_model "
               << target_description_param << cycle_counter;

    system(td_sstream.str().c_str());
  }
}

bool TrajectorySampler::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.02;  // radians
    gripper_joint_positions[1] = 0.02;  // radians
  }
  else
  {
    gripper_joint_positions[0] = -0.01;  // radians
    gripper_joint_positions[1] = -0.01;  // radians
  }

  eef_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = eef_group.move();
  return success;
}

bool TrajectorySampler::OpenGripper()
{
  bool success = OperateGripper(false);
  ROS_INFO("Gripper actuation: Opening %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool TrajectorySampler::CloseGripper()
{
  bool success = OperateGripper(true);
  ROS_INFO("Gripper actuation: Closing %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool TrajectorySampler::SetupCollisionObject(const std::string &object_id,
    const std::string &mesh_path,
    const geometry_msgs::Pose &object_pose,
    moveit_msgs::CollisionObject &collision_object)
{
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

  ROS_DEBUG_STREAM(object_id << " mesh loaded");

  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = object_mesh;

  collision_object.mesh_poses[0].position = object_pose.position;
  collision_object.mesh_poses[0].orientation = object_pose.orientation;

  collision_object.meshes.push_back(object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
}


TrajectorySampler::~TrajectorySampler() {}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_sampler");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  TrajectorySampler plan_sampler(nh);
  return 0;
}
