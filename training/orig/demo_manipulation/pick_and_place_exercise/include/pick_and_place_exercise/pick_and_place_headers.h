/*
 * pick_and_place_headers.h
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#ifndef PICK_AND_PLACE_HEADERS_H_
#define PICK_AND_PLACE_HEADERS_H_

// =============================== ros headers ===============================
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>



// =============================== moveit headers ===============================
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>



// =============================== object manipulation headers ===============================
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>



// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;



// =============================== program constants ===============================
static const std::string GRASP_ACTION_SERVICE = "grasp_execution_action";



// =============================== global variables =====================================
static geometry_msgs::Pose box_pose;
static std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;
static std::vector<geometry_msgs::Pose> tcp_place_poses,wrist_place_poses;



// =============================== ros parameters ===============================
static std::string ARM_GROUP_NAME = "manipulator";
static std::string TCP_LINK_NAME = "tcp_frame";
static std::string WRIST_LINK_NAME = "ee_link";
static std::string WORLD_FRAME_ID= "world_frame";
static std::string HOME_POSE_NAME = "home";
static std::string WAIT_POSE_NAME = "wait";
static std::string TAG_FRAME_ID = "ar_tag";
static tf::Vector3 BOX_SIZE = tf::Vector3(0.1f,0.1f,0.1f);
static tf::Transform BOX_PICK_TF = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(-0.8f,0.2f,BOX_SIZE.getZ()));
static tf::Transform BOX_PLACE_TF = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()));
static double RETREAT_DISTANCE = 0.05f;
static double APPROACH_DISTANCE = 0.05f;

// =============================== Task Functions ===============================
void task_move_to_wait_position(move_group_interface::MoveGroup& move_group);
void task_open_gripper(GraspActionClient& grasp_action_client);
void task_detect_box_pick(tf::TransformListener &tf_listener);
void task_create_pick_moves(tf::TransformListener &tf_listener);
void task_move_through_pick_poses(move_group_interface::MoveGroup& move_group,GraspActionClient& grasp_action_client);
void task_create_place_moves(tf::TransformListener& tf_listener);
void task_move_through_place_poses(move_group_interface::MoveGroup& move_group,GraspActionClient& grasp_action_client);


#endif /* PICK_AND_PLACE_HEADERS_H_ */
