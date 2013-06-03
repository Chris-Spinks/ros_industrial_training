/*
 * task_move_through_place_poses.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place_headers.h>
#include <pick_and_place_exercise/pick_and_place_utilities.h>

void task_move_through_place_poses(move_group_interface::MoveGroup& move_group,GraspActionClient& grasp_action_client)
{
	/*	MOVE ARM THROUGH PLACE POSES
		Goal:
			- Move the robot to each place pose.
			- Open gripper after reaching the target pose
		Hints:
			- Use the methods seen so far such as 'move', 'sendGoal', 'waitForResult' as needed
	Complete code below: */

	// task variables
	object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
	bool success;

	// setting end-effector link and reference frame
	move_group.setEndEffectorLink(WRIST_LINK_NAME);
	move_group.setPoseReferenceFrame(WORLD_FRAME_ID);

	// move the robot to each wrist place pose
	for(unsigned int i = 0; i < wrist_place_poses.size(); i++)
	{
		// set the current place pose as the target
		/* Fill Code: [ use the 'setPoseTarget' method in the 'move_group' object and pass the current pose in 'wrist_pick_poses'] */
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
			return;
		}

		// turn off gripper suction (RELEASE) after reaching target pose
		if(i == 1)
		{
			// set the grasp to open
			/* Fill Code: [ update 'grasp_goal.goal' member] */
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
				exit(1);
			}
		}
	}
}



