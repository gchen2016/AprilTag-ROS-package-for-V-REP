#ifndef STATE_EST_CONTROLLER_H
#define STATE_EST_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
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
		// Callback functions
		void PosCallback(const geometry_msgs::PoseArray::ConstPtr& input_pos_array);
		void IdCallback(const std_msgs::Int32MultiArray::ConstPtr& input_id_array);
		void DesPosCallback(const geometry_msgs::PoseStamped::ConstPtr& input_des_pos);
		void YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw);
		void LeftMotorCallback(const sensor_msgs::JointState::ConstPtr& input_left_state);
		void RightMotorCallback(const sensor_msgs::JointState::ConstPtr& input_right_state);
		void VrepPosCallback(const geometry_msgs::PoseStamped::ConstPtr& input_vrep_pos);

		// Pose functions
		void PoseTransform();
		void UpdateTagMaxtrix();
		void UpdateRobotMatrix();
		void AccumPose();
		void UpdatePose(float inputX, float inputY, float inputYaw);
		void GoToDesPose();

		// Control functions
		void updateAlpha(float dx, float dy, float dth);
		void updateBeta(float dx, float dy, float dth);
		void updateP(float dx, float dy, float dth);
		void setWheel(float v, float w);
		float getYawDiff(float y1, float y2);
		float normTheta(float input_w);


	private:
		ros::Subscriber pos_sub_;
		ros::Subscriber id_sub_;
		ros::Subscriber yaw_sub_;
		ros::Subscriber des_pos_sub_;
		ros::Subscriber left_state_sub_;
		ros::Subscriber right_state_sub_;
		ros::Subscriber vrep_pos_sub_;

		ros::Publisher est_pos_pub_;
		ros::Publisher left_motor_pub_;
		ros::Publisher right_motor_pub_;

		int32_t id;
		Point pos_diff;
		Point pos;
		Point des_pos;
		geometry_msgs::Pose april_pos;
		std::vector<Point> tag_array;
		float left_v;
		float right_v;

		Eigen::Matrix3d world_to_tag;
		Eigen::Matrix3d tag_to_cam;
		Eigen::Matrix3d cam_to_robot;
		Eigen::Matrix3d robot_to_world;

		float alpha;
		float beta;
		float p;

		bool isRandom;
		bool isAprilTag;
	};
}

#endif