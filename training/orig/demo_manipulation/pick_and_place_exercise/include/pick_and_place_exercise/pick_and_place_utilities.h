/*
 * pick_and_place_utilities.h
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#ifndef PICK_AND_PLACE_UTILITIES_H_
#define PICK_AND_PLACE_UTILITIES_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,
		double approach_dis,const tf::Transform &target_tf);

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,
		const std::vector<geometry_msgs::Pose> tcp_poses);

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec);
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt);

// =============================== Config Parameters ===============================
class pick_and_place_config
{
public:
  std::string ARM_GROUP_NAME, TCP_LINK_NAME, WRIST_LINK_NAME, WORLD_FRAME_ID;
  std::string HOME_POSE_NAME, WAIT_POSE_NAME, TAG_FRAME_ID, GRASP_ACTION_SERVICE;
  tf::Vector3 BOX_SIZE;
  tf::Transform BOX_PICK_TF, BOX_PLACE_TF;
  double RETREAT_DISTANCE, APPROACH_DISTANCE;

  pick_and_place_config()
  {
    ARM_GROUP_NAME  = "manipulator";
    TCP_LINK_NAME   = "tcp_frame";
    WRIST_LINK_NAME = "ee_link";
    WORLD_FRAME_ID  = "world_frame";
    HOME_POSE_NAME  = "home";
    WAIT_POSE_NAME  = "wait";
    TAG_FRAME_ID    = "ar_tag";
    GRASP_ACTION_SERVICE = "grasp_execution_action";
    BOX_SIZE        = tf::Vector3(0.1f, 0.1f, 0.1f);
    BOX_PICK_TF     = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(-0.8f,0.2f,BOX_SIZE.getZ()));
    BOX_PLACE_TF    = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()));
    RETREAT_DISTANCE  = 0.05f;
    APPROACH_DISTANCE = 0.05f;
  }

  bool init();
};

#endif /* PICK_AND_PLACE_UTILITIES_H_ */
