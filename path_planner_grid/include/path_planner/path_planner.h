#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <limits>
#include <vector>
#include <utility>


// A* is used for path planning at the first stage

namespace path_planner {

	const unsigned int MAX_UINT = std::numeric_limits<unsigned int>::max();
	const unsigned int FLOOR_LENGTH = 5;

	struct Node {
		int x;
		int y;
		int parent_x;
		int parent_y;
		float fscore;
		float gscore;

		Node(int input_x, int input_y) : x(input_x), y(input_y);

		bool operator==(const Node& n) const {
			return x == n.x && y == n.y;
		}
		void operator=(const Node& n) const {
			x = n.x;
			y = n.y;
			parent_x = n.parent_x;
			parent_y = n.parent_y;
			fscore = n.fscore;
			gscore = n.gscore;
		}
	};
	class PathPlanner {
	public:
		PathPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathPlanner();

	private:
		void TargetCallback(const geometry_msgs::PointStamped::ConstPtr& input_target_point);
		void EstPosCallback(const geometry_msgs::PointStamped::ConstPtr& input_est_point);
		void AStarSearch();
		unsigned int ManDistance(int x1, int y1, int x2, int y2);
		void getNeighbor(vector<Node> &neighbor, int center_x, int center_y);
		float getNeighborDistance(int x1, int y1, int x2, int y2);
		void ConstructPath();

	private:
		ros::Subscriber target_sub_;
		ros::Subscriber est_pos_sub_;
		ros::Publisher path_array_pub_;

		std::pair<int, int> est_pos;
		std::pair<int, int> target_pos;

		std::vector<Node> open_list; // keep open list sorted
		std::vector<Node> close_list;
		// unsigned float gscore_map [FLOOR_LENGTH][FLOOR_LENGTH]; //distance to starting point
		unsigned int hscore_map [FLOOR_LENGTH][FLOOR_LENGTH]; // distance to target point
		Node floor_map [FLOOR_LENGTH][FLOOR_LENGTH];
		vector<std::pair<int, int> > path;
	};
}

#endif