/*
 * create_pick_moves.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>
std::vector<geometry_msgs::Pose> create_pick_moves(tf::TransformListener &tf_listener, geometry_msgs::Pose &box_pose)
{

  /* CREATE PICK MOVES
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
  std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;

  // create tcp pose at box pick
  /* Fill Code: [ use the 'setOrigin' to set the position of 'world_to_tcp_tf'] */
  world_to_tcp_tf.setOrigin(tf::Vector3(box_pose.position.x,box_pose.position.y,cfg.BOX_SIZE.getZ()));
  world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));

  // creating pick poses for tcp
  tcp_pick_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE,cfg.APPROACH_DISTANCE,world_to_tcp_tf);

  // finding transform from tcp to wrist
  tf_listener.lookupTransform(cfg.TCP_LINK_NAME,cfg.WRIST_LINK_NAME,ros::Time(0.0f),tcp_to_wrist_tf);

  // transforming tcp poses to wrist
  /* Fill Code: [ use the 'transform_from_tcp_to_wrist' function and save results into 'wrist_pick_poses'] */
  wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf,tcp_pick_poses);

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses[1].position);

  return wrist_pick_poses;
}


