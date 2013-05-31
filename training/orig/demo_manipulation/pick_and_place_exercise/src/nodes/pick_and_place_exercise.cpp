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
#include <actionlib/client/simple_action_client.h>
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

// object manipulation headers
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>

// global variables
static const std::string GRASP_ACTION_SERVICE = "grasp_execution_action";

// ros parameters
static std::string ARM_GROUP_NAME = "manipulator";
static std::string TCP_LINK_NAME = "tcp_frame";
static std::string WRIST_LINK_NAME = "ee_link";
static std::string WORLD_FRAME_ID= "world_frame";
static std::string HOME_POSE_NAME = "home";
static std::string WAIT_POSE_NAME = "wait";
static std::string TAG_FRAME_ID = "ar_tag";
static tf::Vector3 BOX_SIZE = tf::Vector3(0.1f,0.1f,0.1f);
static tf::Transform BOX_PICK_TF = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(-0.8f,0.2f,BOX_SIZE.getZ()/2.0f));
static tf::Transform BOX_PLACE_TF = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()/2.0f));
static double RETREAT_DISTANCE = 0.05f;
static double APPROACH_DISTANCE = 0.05f;

using namespace tf;

bool read_ros_parameters()
{
	ros::NodeHandle nh("~");
	double w, l, h, x, y, z;

	if(nh.getParam("arm_group_name",ARM_GROUP_NAME)
			&& nh.getParam("tcp_link_name",TCP_LINK_NAME)
			&& nh.getParam("wrist_link_name",WRIST_LINK_NAME)
			&& nh.getParam("world_frame_id",WORLD_FRAME_ID)
			&& nh.getParam("home_pose_name",HOME_POSE_NAME)
			&& nh.getParam("wait_pose_name",WAIT_POSE_NAME)
			&& nh.getParam("tag_frame_id",TAG_FRAME_ID)
			&& nh.getParam("box_width",w)
			&& nh.getParam("box_length",l)
			&& nh.getParam("box_height",h)
			&& nh.getParam("box_place_x",x)
			&& nh.getParam("box_place_y",y)
			&& nh.getParam("box_place_z",z)
			&& nh.getParam("retreat_distance",RETREAT_DISTANCE)
			&& nh.getParam("approach_distance",APPROACH_DISTANCE))
	{
		BOX_SIZE = Vector3(l,w,h);
		BOX_PLACE_TF.setOrigin(Vector3(x,y,z));
		return true;
	}
	else
	{
		return false;
	}

}

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf)
{
	geometry_msgs::Pose start_pose, target_pose, end_pose;
	std::vector<geometry_msgs::Pose> poses;

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

	return poses;

}

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,const std::vector<geometry_msgs::Pose> tcp_poses)
{
	// array for poses of the wrist
	std::vector<geometry_msgs::Pose> wrist_poses;
	wrist_poses.resize(tcp_poses.size());

	// applying transform to each tcp poses
	tf::Transform world_to_wrist_tf, world_to_tcp_tf;
	wrist_poses.resize(tcp_poses.size());
	for(unsigned int i = 0; i < tcp_poses.size(); i++)
	{
		tf::poseMsgToTF(tcp_poses[i],world_to_tcp_tf);
		world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;
		tf::poseTFToMsg(world_to_wrist_tf,wrist_poses[i]);
	}

	return wrist_poses;
}

int main(int argc,char** argv)
{

	/* =================== TASK =====================
		INITIALIZE ROS NODE
		Goal:
			- Observe all steps needed to properly initialize a ros node.
			- Look into the 'read_ros_parameters' function to take notice of the parameters that
				are available for the rest of the program.
		Hints:
		-
	Complete code below: */

	// ros initialization
	ros::init(argc,argv,"pick_and_place_node");
	ros::NodeHandle nh;
	tf::TransformListener tf_listener; // queries tf to find transforms
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// reading parameters
	if(read_ros_parameters())
	{
		ROS_INFO_STREAM("Parameters successfully read");
	}
	else
	{
		ROS_ERROR_STREAM("Parameters not found");
		return 0;
	}

	// moveit interface initialization
	move_group_interface::MoveGroup move_group(ARM_GROUP_NAME);

	// grasp action client initialization
	actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> grasp_action_client(GRASP_ACTION_SERVICE,true);

	// waiting to establish connections
	while(ros::ok() &&
			( !grasp_action_client.waitForServer(ros::Duration(2.0f))))
	{
		ROS_INFO_STREAM("Waiting for servers");
	}

	/* =============== END OF TASK =================== */





	/* =================== TASK =====================
		MOVING ARM TO WAIT POSITION
		Goal:
			- Use the 'move_group' interface to move the robot to the 'wait' target.
			- Verify that the move was completed. Exit on failure

		Hints:
			- The 'WAIT_POSE_NAME' contains the name of the wait target.
			- Once the target is set you can call the 'move' method in order to go to that target.
	Complete code below: */

	// task variables
	bool success; // saves the move result


	move_group.setNamedTarget(WAIT_POSE_NAME);
	success = move_group.move();
	if(success)
	{
		ROS_INFO_STREAM("Move " << WAIT_POSE_NAME<< " Succeeded");
	}
	else
	{
		ROS_INFO_STREAM("Move " << WAIT_POSE_NAME<< " Failed");
		ros::shutdown();
		return 0;
	}
	/* =============== END OF TASK =================== */





	/* =================== TASK =====================
		OPEN GRIPPER
		Goal:
			- Use the grasp action client to open the gripper.
			- Confirm that the gripper was successfully opened.  Exit program on failure;
		Hints:
			-
	Complete code below: */

	// task variables
	object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;

	// send grasp goal to open gripper
	grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
	grasp_action_client.sendGoal(grasp_goal);

	// confirm that gripper opened
	success = grasp_action_client.waitForResult(ros::Duration(4.0f));
	if(success)
	{
		ROS_INFO_STREAM("Gripper opened");
	}
	else
	{
		ROS_ERROR_STREAM("Gripper failure");
		ros::shutdown();
		return 0;
	}

	/* =============== END OF TASK =================== */





	/* =================== TASK =====================
		DETECTING BOX PICK POSE
		Goal:
			- Find the box's pick pose in the world frame using the transform listener.
			- Save the pose into 'box_pose'.

		Hints:
			- tf::poseTFToMsg allows converting transforms into Pose messages

	Complete code below: */

	// task variables
	geometry_msgs::Pose box_pose;
	tf::StampedTransform world_to_box_pick_tf;

	// use transform listener to find the box's pick pose
	tf_listener.lookupTransform(WORLD_FRAME_ID,TAG_FRAME_ID,ros::Time(0.0f),world_to_box_pick_tf);

	// save pose in 'box_pose'
	tf::poseTFToMsg(world_to_box_pick_tf,box_pose);

	/* =============== END OF TASK =================== */






	/* =================== TASK =====================
		CREATE PICK MOVES
		Goal:
			- Set the pose for the tcp at the box pick.
			- Create a tcp poses to be used in between pick moves (Approach, target, retreat).
			- Find transform of the wrist in tcp coordinates
			- Convert tcp pick poses to wrist poses.

		Hints:
			- You can manipulate the 'world_to_tcp_tf' transform through the 'setOrigin' and 'setRotation'.
			- Look into the 'create_manipulation_poses' function and observe how each pick pose is created.
			- Use the 'transform_from_tcp_to_wrist' function and in order to populate the 'wrist_pick_poses' array.
	Complete code below: */

	// task variables
	tf::Transform world_to_tcp_tf;
	std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;
	tf::StampedTransform tcp_to_wrist_tf;

	// create tcp pose at box pick
	world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,BOX_SIZE.getZ()/2.0f));
	world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));

	// creating pick poses for tcp
	tcp_pick_poses = create_manipulation_poses(RETREAT_DISTANCE,APPROACH_DISTANCE,world_to_tcp_tf);

	// finding transform from tcp to wrist
	tf_listener.lookupTransform(TCP_LINK_NAME,WRIST_LINK_NAME,ros::Time(0.0f),tcp_to_wrist_tf);

	// transforming tcp poses to wrist
	wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf,tcp_pick_poses);

	// printing some results
	ROS_INFO_STREAM("tcp position at pick: ["<<world_to_tcp_tf.getOrigin().getX()<<", "
			<<world_to_tcp_tf.getOrigin().getY() <<", "
			<<world_to_tcp_tf.getOrigin().getZ()<<"]");

	tf::Vector3 z_dir = world_to_tcp_tf.getBasis().getColumn(2);
	ROS_INFO_STREAM("tcp z direction at pick: ["<<z_dir.getX()<<", "
			<<z_dir.getY() <<", "
			<<z_dir.getZ()<<"]");

	geometry_msgs::Pose pick_pose = wrist_pick_poses[1];
	ROS_INFO_STREAM("wrist position at pick: ["<<pick_pose.position.x<<", "
			<<pick_pose.position.y <<", "
			<<pick_pose.position.z <<"]");

	/* =============== END OF TASK =================== */




	/* =================== TASK =====================
		MOVE ARM THROUGH PICK POSES
		Goal:
			- Use the 'move_group' object to set the wrist as the end-effector link
			- Use the 'move_group' object to set the world frame as the reference frame for path planning
			- Move the robot to each pick pose.
			- Close gripper after reaching the approach pose

		Hints:
			- The 'move_group' interface has useful method such as 'setEndEffectorLink' and 'setPoseReferenceFrame' that
				can be used to prepare the robot for planning.
			- The 'setPoseTarget' method allows you to set a "pose" as your target to move the robot.

	Complete code below: */

	// set the wrist as the end-effector link
	move_group.setEndEffectorLink(WRIST_LINK_NAME);

	// set world frame as the reference
	move_group.setPoseReferenceFrame(WORLD_FRAME_ID);

	// move the robot to each wrist pick pose
	for(unsigned int i = 0; i < wrist_pick_poses.size(); i++)
	{
		// set the current pose as the target
		move_group.setPoseTarget(wrist_pick_poses[i],WRIST_LINK_NAME);

		// moving arm to current pick pose
		success = move_group.move();

		// verifying move completion
		if(success)
		{
			ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
		}
		else
		{
			ROS_INFO_STREAM("Pick Move " << i <<" Failed");
			ros::shutdown();
			return 0;
		}

		// turn on gripper suction after approach pose
		if(i == 0)
		{
			// set the grasp to close
			grasp_goal.goal= object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;

			// send grasp goal to server
			grasp_action_client.sendGoal(grasp_goal);

			// verify grasp completion
			success = grasp_action_client.waitForResult(ros::Duration(4.0f));
			if(success)
			{
				ROS_INFO_STREAM("Gripper closed");
			}
			else
			{
				ROS_ERROR_STREAM("Gripper failure");
				ros::shutdown();
				return 0;
			}
		}

	}
	/* =============== END OF TASK =================== */





	/* =================== TASK =====================
		CREATE PLACE MOVES
		Goal:
			- Save box place pose in 'box_pose'
			- Set the pose of the tcp at the box place
			- Create a tcp poses to be used in between place moves (Approach, target, retreat).
			- Convert tcp place poses to wrist poses.

		Hints:
			- You can manipulate the 'world_to_tcp_tf' transform through the 'setOrigin' and 'setRotation'.
			- Use the 'create_manipulation_poses' function and in order to create the tcp poses between each place move
			- Use the 'transform_from_tcp_to_wrist' function and in order to populate the 'wrist_place_poses' array.
	Complete code below: */

	// task variables
	std::vector<geometry_msgs::Pose> tcp_place_poses,wrist_place_poses;

	// setting box pose at place
	tf::poseTFToMsg(BOX_PLACE_TF,box_pose);

	// finding tcp pose at box place
	world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,box_pose.position.z));
	world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));

	// creating place poses for tcp
	tcp_place_poses = create_manipulation_poses(RETREAT_DISTANCE,APPROACH_DISTANCE,world_to_tcp_tf);

	// transforming tcp poses to wrist
	wrist_place_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf,tcp_place_poses);

	// printing results
	ROS_INFO_STREAM("tcp position at pick: ["<<world_to_tcp_tf.getOrigin().getX()<<", "
			<<world_to_tcp_tf.getOrigin().getY() <<", "
			<<world_to_tcp_tf.getOrigin().getZ()<<"]");

	geometry_msgs::Pose place_pose = wrist_place_poses[1];
	ROS_INFO_STREAM("wrist position at place: ["<<place_pose.position.x<<", "
			<<place_pose.position.y <<", "
			<<place_pose.position.z <<"]");

	/* =============== END OF TASK =================== */






	/* =================== TASK =====================
		MOVE ARM THROUGH PLACE POSES
		Goal:
			- Move the robot to each place pose.
			- Open gripper after reaching the target pose
		Hints:
			- Use the methods seen so far such as 'move', 'sendGoal', 'waitForResult' as needed
	Complete code below: */

	// setting end-effector link and reference frame
	move_group.setEndEffectorLink(WRIST_LINK_NAME);
	move_group.setPoseReferenceFrame(WORLD_FRAME_ID);

	// move the robot to each wrist place pose
	for(unsigned int i = 0; i < wrist_place_poses.size(); i++)
	{
		// set the current place pose as the target
		move_group.setPoseTarget(wrist_place_poses[i],WRIST_LINK_NAME);

		// moving arm to current place pose
		success = move_group.move();

		if(success)
		{
			ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
		}
		else
		{
			ROS_INFO_STREAM("Place Move " << i <<" Failed");
			ros::shutdown();
			return 0;
		}

		// turn off gripper suction (RELEASE) after reaching target pose
		if(i == 1)
		{
			// set the grasp to open
			grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;

			// send grasp goal to server
			grasp_action_client.sendGoal(grasp_goal);

			// verify grasp completion
			success = grasp_action_client.waitForResult(ros::Duration(4.0f));
			if(success)
			{
				ROS_INFO_STREAM("Gripper opened");
			}
			else
			{
				ROS_ERROR_STREAM("Gripper failure");
				ros::shutdown();
				return 0;
			}
		}
	}

	/* =============== END OF TASK =================== */





	/* =================== TASK =====================
		MOVING ARM TO WAIT POSITION
		Goal:
			- Finish program by moving the arm the the wait position and verify completion.

		Hints:
		-
	Complete code below: */


//	move_group.setNamedTarget(WAIT_POSE_NAME);
//	if(move_group.move())
//	{
//		ROS_INFO_STREAM("Move " << HOME_POSE_NAME<< " Succeeded");
//	}
//	else
//	{
//		ROS_INFO_STREAM("Move " << HOME_POSE_NAME<< " Failed");
//		ros::shutdown();
//		return 0;
//	}


	/* =============== END OF TASK =================== */

	return 0;
}
