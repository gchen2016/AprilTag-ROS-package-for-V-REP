#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <path_follow/path_util.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

namespace path_follow {
	class PathFollow {
	public:
		PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathFollow();

	private:
		void EstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_est_pose);
		void PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path);
		void YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw);
		void ComputeDesPose();

	private:
		ros::Subscriber est_pos_sub_;
		ros::Subscriber yaw_sub_;
		ros::Subscriber path_sub_;
		ros::Publisher des_pos_pub_;

		geometry_msgs::PoseStamped est_pos;
		nav_msgs::Path planned_path;
		float est_yaw;
		int yaw_int;
	};
}

#endif