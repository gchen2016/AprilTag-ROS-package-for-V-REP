#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <path_plan/grid_graph.h>

#include <vector>
#include <utility>
#include <limits>

namespace path_plan {
	// const int MAX_INT = std::numeric_limits<int>::max();
	const int MAX_INT = 8888;

	// struct AStarNode {
	// 	int id;
	// 	int fscore;
	// };

	class PathPlan {
	public:
		PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathPlan();

	private:
		void EstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_est_pose);
		void TargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_target_pose);
		void YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw);
		void AStarSearch();
		int ManDistance(int x1, int y1, int t1, int x2, int y2, int t2);
		int ThetaDistance(int t1, int t2);
		void ConstructPath(int target_id);

	private:
		ros::Subscriber est_pos_sub_;
		ros::Subscriber target_pos_sub_;
		ros::Subscriber yaw_sub_;
		ros::Publisher path_pub_;

		geometry_msgs::PoseStamped est_pos;
		geometry_msgs::PoseStamped target_pos;
		
		int yaw_int;

		GridGraph *graph;
		std::vector<geometry_msgs::PoseStamped> path;

		int parentIdArray[FLOOR_LENGTH * FLOOR_LENGTH * DIRECTION];
		// distance to target point
		int hscoreArray[FLOOR_LENGTH * FLOOR_LENGTH * DIRECTION];
		// distance to starting point
		int gscoreArray[FLOOR_LENGTH * FLOOR_LENGTH * DIRECTION];
		// h + g score
		int fscoreArray[FLOOR_LENGTH * FLOOR_LENGTH * DIRECTION];
		
		std::vector<int> openList;
		std::vector<int> closedList;
	};
}

#endif