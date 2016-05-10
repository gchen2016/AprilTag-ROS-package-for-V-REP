#include <path_follow/path_follower.h>

#include <vector>
#include <algorithm>

namespace path_follow {
	const int FLOOR_LENGTH = 5;

	struct PathPoint {
		int x;
		int y;
		int yaw;
		int distance;
	};

	inline bool operator<(const PathPoint &a, const PathPoint &b) {
		return a.distance < b.distance;
	}

	PathFollow::PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		// subscriptions
		est_pos_sub_ = nh.subscribe("state_est_pos", 1, &PathFollow::EstPoseCallback, this);
		yaw_sub_ = nh.subscribe("tag_detections_yaw", 1, &PathFollow::YawCallback, this);
		path_sub_ = nh.subscribe("planned_path", 1, &PathFollow::PlannedPathCallback, this);

		// publications
		des_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("des_pos", 1);

	}

	PathFollow::~PathFollow() {
		est_pos_sub_.shutdown();
		yaw_sub_.shutdown();
		path_sub_.shutdown();
	}

	void PathFollow::EstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_est_pose) {
		est_pos = *input_est_pose;
		est_pos.pose.position.x += FLOOR_LENGTH/2;
		est_pos.pose.position.y += FLOOR_LENGTH/2;
	}

	void PathFollow::PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path) {
		planned_path = *input_path;
		ComputeDesPose();
	}
	void PathFollow::YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw) {
		std_msgs::Float32MultiArray yaw_array = *input_yaw;
		for(int i=0; i < yaw_array.data.size(); i++) {
			if(yaw_array.data[i] != 0)
				est_yaw = yaw_array.data[i];
		}
		yaw_int = path_util::getYawEnum(est_yaw);
	}

	void PathFollow::ComputeDesPose() {
		PathPoint current_point = {
			(int)round(est_pos.pose.position.x),
			(int)round(est_pos.pose.position.y),
			yaw_int,
			0
		};

		// loop in nav path to get the nearest point
		std::vector<PathPoint> path_array;
		for(int i=0; i < planned_path.poses.size(); i++) {
			geometry_msgs::PoseStamped tmp_pos = planned_path.poses[i];
			PathPoint tmp_point = {
				(int)round(tmp_pos.pose.position.x),
				(int)round(tmp_pos.pose.position.y),
				(int)round(tmp_pos.pose.position.z),
				0
			};
			tmp_point.distance = path_util::ManDistance(current_point.x, current_point.y, current_point.yaw, tmp_point.x, tmp_point.y, tmp_point.yaw);

			path_array.push_back(tmp_point);
		}

		// sort path_array on smallest distance
		std::sort(path_array.begin(), path_array.end());

		geometry_msgs::PoseStamped des_pos;
		PathPoint des_point = path_array.front();
		des_pos.pose.position.x = (float)des_point.x;
		des_pos.pose.position.y = (float)des_point.y;
		des_pos.pose.position.z = path_util::getYawFloat(des_point.yaw);
		
		// publish
		des_pos_pub_.publish(des_pos);
	}
}