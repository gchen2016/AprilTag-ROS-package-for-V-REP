#ifndef STATE_EST_CONTROLLER_H
#define STATE_EST_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>

namespace state_est_control {
	struct Point {
		float x;
		float y;
		float yaw;
		Point(float inputX, float inputY, float inputYaw) : x(inputX), y(inputY), yaw(inputYaw) {}
	};

	class StateEstControl {
	public:
		StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~StateEstControl();

	private:
		void PosCallback(const geometry_msgs::PoseArray::ConstPtr& input_pos_array);
		void IdCallback(const std_msgs::Int32MultiArray::ConstPtr& input_id_array);
		void DesPosCallback(const geometry_msgs::PoseStamped::ConstPtr& input_des_pos);
		void YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw);
		void updatePose();
		void GoToDesPose();
		void updateAlpha(float dx, float dy, float dth);
		void updateBeta(float dx, float dy, float dth);
		void updateP(float dx, float dy, float dth);
		void setWheel(float v, float w);


	private:
		ros::Subscriber pos_sub_;
		ros::Subscriber id_sub_;
		ros::Subscriber yaw_sub_;
		ros::Subscriber des_pos_sub_;
		ros::Publisher est_pos_pub_;
		ros::Publisher left_motor_pub_;
		ros::Publisher right_motor_pub_;

		int32_t id;
		Point pos_diff;
		Point pos;
		Point des_pos;
		std::vector<Point> tag_array;

		float alpha;
		float beta;
		float p;
	};
}

#endif