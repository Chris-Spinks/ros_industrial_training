/*
 * task_create_pick_moves.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place_headers.h>
#include <pick_and_place_exercise/pick_and_place_utilities.h>

void task_create_pick_moves(tf::TransformListener &tf_listener)
{
	/*	CREATE PICK MOVES
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
	tf::StampedTransform tcp_to_wrist_tf;

	// create tcp pose at box pick
	/* Fill Code: [ use the 'setOrigin' to set the position of 'world_to_tcp_tf'] */
	world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,BOX_SIZE.getZ()));
	world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));

	// creating pick poses for tcp
	tcp_pick_poses = create_manipulation_poses(RETREAT_DISTANCE,APPROACH_DISTANCE,world_to_tcp_tf);

	// finding transform from tcp to wrist
	tf_listener.lookupTransform(TCP_LINK_NAME,WRIST_LINK_NAME,ros::Time(0.0f),tcp_to_wrist_tf);

	// transforming tcp poses to wrist
	/* Fill Code: [ use the 'transform_from_tcp_to_wrist' function and save results into 'wrist_pick_poses'] */
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
}


