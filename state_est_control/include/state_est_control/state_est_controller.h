#ifndef STATE_EST_CONTROLLER_H
#define STATE_EST_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <vector>

namespace state_est_control {
	struct Point {
		float x;
		float y;
		Point(float inputX, float inputY) : x(inputX), y(inputY) {}
	};

	class StateEstControl {
	public:
		StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~StateEstControl();

	private:
		void PosCallback(const geometry_msgs::PoseArray::ConstPtr& input_pos_array);
		void IdCallback(const std_msgs::Int32MultiArray::ConstPtr& input_id_array);
		void updatePose();

	private:
		ros::Subscriber pos_sub_;
		ros::Subscriber id_sub_;
		ros::Publisher est_pos_pub_;
		int32_t id;
		Point pos_diff;
		Point pos;
		std::vector<Point> tag_array;

	};
}

#endif