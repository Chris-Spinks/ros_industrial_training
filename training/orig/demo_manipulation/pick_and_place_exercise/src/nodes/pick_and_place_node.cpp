/*
 * Manipulation Lab
 * pick_and_place_node.cpp
 *
 *  Created on: May 21, 2013
 *      Author: ros developer 
 */


#include <string.h>

// ros headers
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

// moveit headers
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <manipulation_msgs/Grasp.h>

// global variables
static const std::string PLANNING_SCENE_TOPIC = "planning_scene";
static const std::string COLLISION_OBJECT_TOPIC = "collision_object";
static const std::string ARM_GROUP_NAME = "manipulator";
static const std::string TCP_LINK_NAME = "tcp_frame";
static const std::string WRIST_LINK_NAME = "ee_link";
static const std::string END_EFFECTOR_NAME = "vacuum_gripper";
static const std::string WORLD_FRAME_ID= "world_frame";
static const std::string COLLISION_OBJECT_ID = "tagged_box";
static const tf::Vector3 BOX_SIZE = tf::Vector3(0.1f,0.1f,0.1f);
static const tf::Transform BOX_START_TF = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(-0.8f,0.2f,BOX_SIZE.getZ()/2.0f));
static const tf::Transform BOX_END_TF = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()/2.0f));
static const double RETREAT_DISTANCE = 0.05f;
static const double APPROACH_DISTANCE = 0.05f;

using namespace tf;

void create_manipulation_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf,std::vector<geometry_msgs::Pose> &poses)
{
	geometry_msgs::Pose start_pose, target_pose, end_pose;

	// creating start pose by applying a translation along +z by approach distance
	tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*target_tf,start_pose);

	// converting target pose
	tf::poseTFToMsg(target_tf,target_pose);

	// creating end pose by applying a translation along +z by retreat distance
	tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,retreat_dis))*target_tf,end_pose);

	poses.clear();
	poses.push_back(start_pose);
	poses.push_back(target_pose);
	poses.push_back(end_pose);


}

int main(int argc,char** argv)
{

	/* =================================================
	 * ROS SETUP
	   ================================================= */

	// ros initialization
	ros::init(argc,argv,"pick_and_place_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2);
	tf::TransformListener tf_listener;
	spinner.start();

	// moveit interface
	move_group_interface::MoveGroup move_group(ARM_GROUP_NAME);

	// box pose
	geometry_msgs::Pose box_pose;

	/* =================================================
	 * PICK MOVE SETUP
	   ================================================= */
	// poses for pick move
	std::vector<geometry_msgs::Pose> pick_poses;

	// transform for solving wrist pose in world
	tf::StampedTransform tcp_to_wrist_tf;
	tf::Transform world_to_wrist_tf, world_to_tcp_tf = tf::Transform::getIdentity();

	// finding transform from wrist to tcp
	tf_listener.lookupTransform(TCP_LINK_NAME,WRIST_LINK_NAME,ros::Time(0.0f),tcp_to_wrist_tf);

	// setting box pose at pick
	tf::poseTFToMsg(BOX_START_TF,box_pose);

	// resolving wrist pose relative to world frame at pick
	world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,BOX_SIZE.getZ()));
	world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));
	world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;

	// creating pick poses
	create_manipulation_poses(RETREAT_DISTANCE,APPROACH_DISTANCE,world_to_wrist_tf,pick_poses);

	ROS_INFO_STREAM("tcp position at pick: ["<<world_to_tcp_tf.getOrigin().getX()<<", "
			<<world_to_tcp_tf.getOrigin().getY() <<", "
			<<world_to_tcp_tf.getOrigin().getZ()<<"]");

	tf::Vector3 z_dir = world_to_tcp_tf.getBasis().getColumn(2);
	ROS_INFO_STREAM("tcp z direction at pick: ["<<z_dir.getX()<<", "
			<<z_dir.getY() <<", "
			<<z_dir.getZ()<<"]");

	geometry_msgs::Pose pick_pose = pick_poses[1];
	ROS_INFO_STREAM("wrist position at pick: ["<<pick_pose.position.x<<", "
			<<pick_pose.position.y <<", "
			<<pick_pose.position.z <<"]");


	/* =================================================
	 * PLACE MOVE SETUP
	   ================================================= */
	// poses for place move
	std::vector<geometry_msgs::Pose> place_poses;

	// setting box pose at place
	tf::poseTFToMsg(BOX_END_TF,box_pose);

	// resolving wrist pose in world at place
	world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,BOX_SIZE.getZ()));
	world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));
	world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;

	// creating place poses
	create_manipulation_poses(RETREAT_DISTANCE,APPROACH_DISTANCE,world_to_wrist_tf,place_poses);

	ROS_INFO_STREAM("tcp position at pick: ["<<world_to_tcp_tf.getOrigin().getX()<<", "
			<<world_to_tcp_tf.getOrigin().getY() <<", "
			<<world_to_tcp_tf.getOrigin().getZ()<<"]");

	geometry_msgs::Pose place_pose = place_poses[1];
	ROS_INFO_STREAM("wrist position at place: ["<<place_pose.position.x<<", "
			<<place_pose.position.y <<", "
			<<place_pose.position.z <<"]");


	/* =================================================
	 * MOVING ARM THROUGH PICK POSES
	   ================================================= */

	move_group.setEndEffectorLink(WRIST_LINK_NAME);
	move_group.setPoseReferenceFrame(WORLD_FRAME_ID);

	for(int i = 0; i < pick_poses.size(); i++)
	{
		move_group.setPoseTarget(pick_poses[i],WRIST_LINK_NAME);
		if(move_group.move())
		{
			ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
		}
		else
		{
			ROS_INFO_STREAM("Pick Move " << i <<" Failed");
			ros::shutdown();
			return 0;
		}
	}

	/* =================================================
	 * MOVING ARM THROUGH PLACE POSES
	   ================================================= */

	move_group.setEndEffectorLink(WRIST_LINK_NAME);
	move_group.setPoseReferenceFrame(WORLD_FRAME_ID);

	for(int i = 0; i < place_poses.size(); i++)
	{
		move_group.setPoseTarget(place_poses[i],WRIST_LINK_NAME);
		if(move_group.move())
		{
			ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
		}
		else
		{
			ROS_INFO_STREAM("Place Move " << i <<" Failed");
			ros::shutdown();
			return 0;
		}
	}

	return 0;
}
