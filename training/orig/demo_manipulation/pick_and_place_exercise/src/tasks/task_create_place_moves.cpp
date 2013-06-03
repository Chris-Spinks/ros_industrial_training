/*
 * task_create_place_moves.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place_headers.h>
#include <pick_and_place_exercise/pick_and_place_utilities.h>

void task_create_place_moves(tf::TransformListener& tf_listener)
{
	/*	CREATE PLACE MOVES
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
	tf::Transform world_to_tcp_tf;
	tf::StampedTransform tcp_to_wrist_tf;

	// setting 'box_pose' at place
	tf::poseTFToMsg(BOX_PLACE_TF,box_pose);


	// finding tcp pose at box place
	/* Fill Code: [ use the 'setOrigin' to set the position of 'world_to_tcp_tf'] */
	world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,box_pose.position.z));

	/* Fill Code: [ use the 'setRotation' to set the orientation of 'world_to_tcp_tf'] */
	world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));

	// creating place poses for tcp
	/* Fill Code: [ use the 'create_manipulation_poses' and save results to 'tcp_place_poses'] */
	tcp_place_poses = create_manipulation_poses(RETREAT_DISTANCE,APPROACH_DISTANCE,world_to_tcp_tf);


	// finding transform from tcp to wrist
	/* Fill Code: [ use the 'lookupTransform' method in the transform listener] */
	tf_listener.lookupTransform(TCP_LINK_NAME,WRIST_LINK_NAME,ros::Time(0.0f),tcp_to_wrist_tf);


	// transforming tcp poses to wrist
	wrist_place_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf,tcp_place_poses);

	// printing results
	ROS_INFO_STREAM("tcp position at place: ["<<world_to_tcp_tf.getOrigin().getX()<<", "
			<<world_to_tcp_tf.getOrigin().getY() <<", "
			<<world_to_tcp_tf.getOrigin().getZ()<<"]");

	geometry_msgs::Pose place_pose = wrist_place_poses[1];
	ROS_INFO_STREAM("wrist position at place: ["<<place_pose.position.x<<", "
			<<place_pose.position.y <<", "
			<<place_pose.position.z <<"]");
}

