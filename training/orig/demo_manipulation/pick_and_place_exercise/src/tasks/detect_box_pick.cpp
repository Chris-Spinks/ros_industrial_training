/*
 * detect_box_pick.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

geometry_msgs::Pose detect_box_pick(tf::TransformListener &tf_listener)
{
  /* DETECTING BOX PICK POSE
    Goal:
      - Find the box's pick pose in the world frame using the transform listener.
      - Save the pose into 'box_pose'.

    Hints:
      - tf::poseTFToMsg allows converting transforms into Pose messages

  Complete code below: */

  // task variables
  tf::StampedTransform world_to_box_pick_tf;
  geometry_msgs::Pose box_pose;

  // use transform listener to find the box's pick pose
  /* Fill Code: [ use the 'lookupTransform' method in the transform listener] */
  tf_listener.lookupTransform(cfg.WORLD_FRAME_ID,cfg.TAG_FRAME_ID,ros::Time(0.0f),world_to_box_pick_tf);

  // save pose in 'box_pose'
  /* Fill Code: [ use the 'tf::poseTFToMsg' to save a transform into a pose ] */
  tf::poseTFToMsg(world_to_box_pick_tf,box_pose);

  return box_pose;
}

