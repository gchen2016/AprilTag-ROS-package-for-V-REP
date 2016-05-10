#include <path_plan/path_planner.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <algorithm>

namespace path_plan {

	PathPlan::PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh) : yaw_int(-1) {

		graph = new GridGraph();

		// init gscore
		for(int i=0; i < (FLOOR_LENGTH * FLOOR_LENGTH * DIRECTION); i++)
			gscoreArray[i] = MAX_INT;

		// subscriptions
		est_pos_sub_ = nh.subscribe("state_est_pos", 1, &PathPlan::EstPoseCallback, this);
		target_pos_sub_ = nh.subscribe("target_pos", 1, &PathPlan::TargetPoseCallback, this);
		yaw_sub_ = nh.subscribe("tag_detections_yaw", 1, &PathPlan::YawCallback, this);

		// publications
		path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1);
	}

	PathPlan::~PathPlan() {
		est_pos_sub_.shutdown();
		target_pos_sub_.shutdown();
		yaw_sub_.shutdown();
	}

	void PathPlan::EstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_est_pose) {
		est_pos = *input_est_pose;
		est_pos.pose.position.x += FLOOR_LENGTH/2;
		est_pos.pose.position.y += FLOOR_LENGTH/2;
		AStarSearch();
	}

	void PathPlan::TargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_target_pose) {
		target_pos = *input_target_pose;
		AStarSearch();
	}

	void PathPlan::YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw) {
		std_msgs::Float32MultiArray yaw_array = *input_yaw;
		float yaw;
		for(int i=0; i < yaw_array.data.size(); i++) {
			if(yaw_array.data[i] != 0)
				yaw = yaw_array.data[i];
		}
		yaw_int = (int)round((yaw + 3.1415926) / 1.5707963);
	}

	void PathPlan::AStarSearch() {
		int l = FLOOR_LENGTH;
		int d = DIRECTION;
		int target_id = 0;

		if(yaw_int == -1)
			return;

		// add starting point
		printf("x: %d, y: %d, yaw: %d\n", (int)round(est_pos.pose.position.x), (int)round(est_pos.pose.position.y), yaw_int);
		int start_id = ((int)round(est_pos.pose.position.x))*l*d + ((int)round(est_pos.pose.position.y))*d + yaw_int;
		printf("Start id: %d\n", start_id);
		Node *start_node = graph->nodeList[start_id];
		Node *target_node = graph->nodeList[target_id];
		parentIdArray[start_id] = start_id;

		hscoreArray[start_id] = ManDistance(start_node->x, start_node->y, start_node->theta, target_node->x, target_node->y, target_node->theta);
		printf("Start id hscore: %d\n", hscoreArray[start_id]);
		hscoreArray[target_id] = 0;
		gscoreArray[start_id] = 0;
		gscoreArray[target_id] = hscoreArray[start_id];

		fscoreArray[start_id] = hscoreArray[start_id];
		openList.push_back(start_id);


		while(openList.size() != 0) {
			// std::sort(openList.begin(), openList.end(), [](const int& a, const int& b) { 
			// 	if (fscoreArray[a] < fscoreArray[b])
			// 		return true;
			// 	else
			// 		return false;
			// 	// return a < b;
			// });
			int f_lowest_id = openList[0];
			for (int i=0; i < openList.size(); i++) {
				if(fscoreArray[openList[i]] < fscoreArray[f_lowest_id]) {
					f_lowest_id = openList[i];
				}
			}

			// int current_id = openList.front(); // lowest fscore
			int current_id = f_lowest_id;

			openList.erase(std::remove(openList.begin(), openList.end(), f_lowest_id), openList.end());

			//construc path if target reached
			if(hscoreArray[current_id] == 0) {
				ConstructPath(target_id);
				break;
			}

			closedList.push_back(current_id);

			// calculate adj nodes
			Node *current_node = graph->nodeList[current_id];
			// printf("Current node x: %d, y: %d, yaw: %d\n", current_node->x, current_node->y, current_node->theta);
			for(int i=0; i < current_node->edgeList.size(); i++) {
				Node *adj_node = current_node->edgeList[i].dstNode;
				// printf("Adj node x: %d, y: %d, yaw: %d\n", adj_node->x, adj_node->y, adj_node->theta);

				// check is closed
				if(std::find(closedList.begin(), closedList.end(), adj_node->id) != closedList.end())
					continue;

				// compute g score
				int tmp_gscore = gscoreArray[current_id] + current_node->edgeList[i].cost;

				// insert into open list if not
				if(std::find(openList.begin(), openList.end(), adj_node->id) == openList.end()) {
					openList.push_back(adj_node->id);
					hscoreArray[adj_node->id] = ManDistance(adj_node->x, adj_node->y, adj_node->theta, target_node->x, target_node->y, target_node->theta);
					parentIdArray[adj_node->id] = current_node->id;
				}

				// check if re-parent needed
				// update
				if(tmp_gscore < gscoreArray[adj_node->id]) {
					parentIdArray[adj_node->id] = current_id;
					gscoreArray[adj_node->id] = tmp_gscore;
				}
				fscoreArray[adj_node->id] = gscoreArray[adj_node->id] + hscoreArray[adj_node->id];

			}

		}

	}

	int PathPlan::ThetaDistance(int t1, int t2) {
		return (abs(t1 - t2) < 3) ? abs(t1 - t2) : (abs(t1 -t2) - 2);
	}

	int PathPlan::ManDistance(int x1, int y1, int t1, int x2, int y2, int t2) {
		return (abs(x1 - x2) + abs(y1 - y2) + ThetaDistance(t1, t2));
	}

	void PathPlan::ConstructPath(int target_id) {
		int current_id = target_id;
		std::vector<int> result_list;
		while(parentIdArray[current_id] != current_id) {
			result_list.push_back(current_id);
			current_id = parentIdArray[current_id];
		}

		// publish path result
		nav_msgs::Path planned_path;
		for(int i=0; i < result_list.size(); i++) {
			int index = result_list[i];
			geometry_msgs::PoseStamped g;
			g.pose.position.x = (float)graph->nodeList[index]->x;
			g.pose.position.y = (float)graph->nodeList[index]->y;
			g.pose.position.z = (float)graph->nodeList[index]->theta;
			printf("Construct id x: %f, y: %f, t: %f\n", g.pose.position.x, g.pose.position.y, g.pose.position.z);
			planned_path.poses.push_back(g);
		}
		path_pub_.publish(planned_path);
	}
}