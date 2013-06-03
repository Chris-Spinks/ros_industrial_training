/*
 * Manipulation Lab
 * pick_and_place_node.cpp
 *
 *  Created on: May 21, 2013
 *      Author: ros developer 
 */

// =============================== pick and place headers ===============================
#include <pick_and_place_exercise/pick_and_place_headers.h>
#include <pick_and_place_exercise/pick_and_place_utilities.h>

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{

	/* =========================================================================================*/
	/*	INITIALIZING ROS NODE
		Goal:
			- Observe all steps needed to properly initialize a ros node.
			- Look into the 'read_ros_parameters' function to take notice of the parameters that
				are available for the rest of the program. */
	/* =========================================================================================*/

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
	GraspActionClient grasp_action_client(GRASP_ACTION_SERVICE,true);

	// waiting to establish connections
	while(ros::ok() &&
			 !grasp_action_client.waitForServer(ros::Duration(2.0f)))
	{
		ROS_INFO_STREAM("Waiting for servers");
	}


	/* =================== TASK 1 =====================*/
	task_move_to_wait_position(move_group);



	/* =================== TASK 2 =====================*/
	task_open_gripper(grasp_action_client);



	/* =================== TASK 3 ===================== */
	task_detect_box_pick(tf_listener);



	/* =================== TASK 4 =====================*/
	task_create_pick_moves(tf_listener);



	/* =================== TASK 5 ===================== */
	task_move_through_pick_poses(move_group,grasp_action_client);



	/* =================== TASK 6 =====================*/
	task_create_place_moves(tf_listener);



	/* =================== TASK 7 =====================*/
	task_move_through_place_poses(move_group,grasp_action_client);


	/* =================== TASK 8 =====================*/
	task_move_to_wait_position(move_group);

	return 0;
}
