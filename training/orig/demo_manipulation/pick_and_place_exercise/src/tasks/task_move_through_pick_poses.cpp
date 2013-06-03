/*
 * task_move_through_pick_poses.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place_headers.h>
#include <pick_and_place_exercise/pick_and_place_utilities.h>

void task_move_through_pick_poses(move_group_interface::MoveGroup& move_group,GraspActionClient& grasp_action_client)
{
	/*	MOVE ARM THROUGH PICK POSES
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

	// task variables
	object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
	bool success;

	// set the wrist as the end-effector link
	/* Fill Code: [ use the 'setEndEffectorLink' in the 'move_group' object] */
	move_group.setEndEffectorLink(WRIST_LINK_NAME);

	// set world frame as the reference
	/* Fill Code: [ use the 'setPoseReferenceFrame' in the 'move_group' object] */
	move_group.setPoseReferenceFrame(WORLD_FRAME_ID);

	// move the robot to each wrist pick pose
	for(unsigned int i = 0; i < wrist_pick_poses.size(); i++)
	{
		// set the current pose as the target
		/* Fill Code: [ use the 'setPoseTarget' method in the 'move_group' object and pass the current pose in 'wrist_pick_poses'] */
		move_group.setPoseTarget(wrist_pick_poses[i],WRIST_LINK_NAME);

		// moving arm to current pick pose
		/* Fill Code: [ use the 'move' method in the 'move_group' object and save the result in the 'success' variable] */
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
			return;
		}

		// turn on gripper suction after approach pose
		if(i == 0)
		{
			// set the grasp to close
			grasp_goal.goal= object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;

			// send grasp goal to server
			/* Fill Code: [ use the 'sendGoal' method of the grasp client to open gripper] */
			grasp_action_client.sendGoal(grasp_goal);


			// verify grasp completion
			/* Fill Code: [ use the 'waitForResult' method to check and save result in success variable] */
			success = grasp_action_client.waitForResult(ros::Duration(4.0f));

			if(success)
			{
				ROS_INFO_STREAM("Gripper closed");
			}
			else
			{
				ROS_ERROR_STREAM("Gripper failure");
				exit(1);
			}
		}

	}
}

