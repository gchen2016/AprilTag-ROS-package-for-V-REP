#include <path_planner/path_planner.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <algorithm>

namespace path_planner {
	PathPlanner::PathPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh) : est_pos(std::make_pair(0, 0)), target_pos(std::make_pair(0, 0)) {

		// init floor map
		for(int i=0; i < FLOOR_LENGTH; i++) {
			for(int j=0; j < FLOOR_LENGTH; j++) {
				floor_map[i][j].x = i;
				floor_map[i][j].y = j;
				floor_map[i][j].gscore = MAX_UINT;
			} // end for j
		} // end for i

		// subscriptions
		target_sub_ = nh.subscribe("target_pose", 1, &PathPlanner::TargetCallback, this);
		est_pos_sub_ = nh.subscribe("state_est_pos", 1, &PathPlanner::EstPosCallback, this);

		// publications
		path_array_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1);
	}

	PathPlanner::~PathPlanner() {
		target_sub_.shutdown();
		est_pos_sub_.shutdown();
	}

	void PathPlanner::TargetCallback(const geometry_msgs::PointStamped::ConstPtr& input_target_point) {
		geometry_msgs::PointStamped target_point = *input_target_point;

		target_pos = std::make_pair((int)target_point.point.x, (int)target_point.point.y);
		AStarSearch();
	}

	void PathPlanner::EstPosCallback(const geometry_msgs::PointStamped::ConstPtr& input_est_point) {
		geometry_msgs::PointStamped est_point = *input_est_point;

		est_pos = std::make_pair((int)est_point.point.x, (int)est_point.point.y);
	}

	void PathPlanner::AStarSearch() {
		if(est_point == std::make_pair(0, 0) || target_point == std::make_pair(0, 0))
			return;

		// 1 step: compute H score map
		for(int i=0; i < FLOOR_LENGTH; i++) {
			for(int j=0; j < FLOOR_LENGTH; j++) {
				// H score: Distance to target point
				hscore_map[i][j] = ManDistance(target_pos.first, target_pos.second, i, j);
			} // end for j
		} // end for i

		// 2 step: add starting point
		floor_map[est_pos.first][est_pos.second].parent_x = est_pos.first;
		floor_map[est_pos.first][est_pos.second].parent_y = est_pos.second;
		open_list.push_back(floor_map[est_pos.first][est_pos.second]);

		// 3 step: a* search
		// Loop through open list 
		// Keep open_list sorted on fscore
		while(open_list.size() != 0) {
			// choose the one with lowest fscore
			Node current = open_list.front();

			// construct path if target reached
			if((current.x == target_pos.x) && (current.y == target_pos.y)) {
				ConstructPath();
				return;
			}

			open_list.erase(open_list.begin());
			closed_list.push_back(current);

			// calculate neighbor list
			std::vector<Node> neighbor_list;
			getNeighbor(neighbor_list, current.x, current.y);

			// 4 step: loop in neighbors
			for(std::vector<Node>::iterator n_it = neighbor_list.begin(); n_it != neighbor_list.end(); n_it++) {
				// ignore if in close list
				if(std::find(closed_list.begin(), closed_list.end(), *n_it) != closed_list.end()) 
					continue;

				// compute g score : distance to starting point
				float tmp_gscore = current.gscore + getNeighborDistance(current.x, current.y, *n_it.x, *n_it.y);

				// check if re-parent needed
				// update
				if(tmp_gscore < *n_it.gscore) {
					*n_it.gscore = tmp_gscore;
					*n_it.parent_x = current.x;
					*n_it.parent_y = current.y;
					*n_it.fscore = *n_it.gscore + hscore_map[*n_it.x][*n_it.y];

					floor_map[*n_it.x][*n_it.y] = *n_it;
				}

				// insert into open list if not in
				// update if in
				std::vector<Node> tmp_it = std::find(open_list.begin(), open_list.end(), *n_it);
				if(tmp_it == open_list.end()) 
					open_list.push_back(*n_it);
				else
					*tmp_it = *n_it;

			} // end for

		}
	}

	unsigned int PathPlanner::ManDistance(int x1, int y1, int x2, int y2) {
		return (abs(x1 - x2) + abs(y1 - y2));
	}

	void PathPlanner::getNeighbor(vector<Node> &neighbor, int center_x, int center_y) {
		int start_x = (center_x -1 < 0) ? center_x : center_x -1;
		int end_x = (center_x +1 > FLOOR_LENGTH) ? center_x : center_x +1;
		int start_y = (center_y -1 < 0) ? center_y ? center_y -1;
		int end_y = (center_y +1 > FLOOR_LENGTH) ? center_y : center_y +1;

		for(int tmpx = start_x; tmpx <= end_x; tmpx++) {
			for(int tmpy = start_y; tmpy <= end_y; tmpy++) {
				if((tmpx != center_x) || (tmpy != center_y))
					neighbor.push_back(floor_map[tmpx][tmpy]);
			} // end for y
		} // end for x
	}

	float PathPlanner::getNeighborDistance(int x1, int y1, int x2, int y2) {
		if ((abs(x1 - x2) + abs(y1 - y2)) == 1)
			return 1
		else if ((abs(x1 - x2) + abs(y1 - y2)) == 2)
			return 1.414;
		else {
			printf("Error neighbor\n");
			return 0;
		}
	}

	void PathPlanner::ConstructPath() {
		Node current = floor_map[target_pos.first][target_pos.second];
		while(current.parent_x != current.x || current.parent_y != current.y) {
			path.push_back(std::make_pair(current.x, current.y));
			current = floor_map[current.parent_x][current.parent_y];
		}
	}
}