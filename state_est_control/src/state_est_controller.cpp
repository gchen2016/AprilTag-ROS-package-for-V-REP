#include <state_est_control/state_est_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <XmlRpcException.h>

#include <math.h>

const int FLOOR_LENGTH = 5;

namespace state_est_control {
	const float CAR_WIDTH = 0.1;
	const float WHEEL_RADIUS = 0.04;

	StateEstControl::StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh) : pos_diff(Point(0, 0, 0)), pos(Point(0, 0, 0)) {

		// init tag array
		// const int HALF_LENGTH = FLOOR_LENGTH / 2;
		for(int y=0; y < FLOOR_LENGTH; y++) {
			for(int x=0; x < FLOOR_LENGTH; x++)
				tag_array.push_back(Point(x, y, 0));
		} 


		// subscriptions
		pos_sub_ = nh.subscribe("tag_detections_pose", 1, &StateEstControl::PosCallback, this);
		id_sub_ = nh.subscribe("tag_detections_id", 1, &StateEstControl::IdCallback, this);
		des_pos_sub_ = nh.subscribe("des_pos", 1, &StateEstControl::DesPosCallback, this);
		yaw_sub_ = nh.subscribe("tag_detections_yaw", 1, &StateEstControl::Ya)

		// publications
		est_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("state_est_pos", 1);
		left_motor_pub_ = nh.advertise<std_msgs::Float32>("left_motor", 1);
		right_motor_pub_ = nh.advertise<std_msgs::Float32>("right_motor", 1);
	}

	StateEstControl::~StateEstControl() {
		pos_sub_.shutdown();
		id_sub_.shutdown();
		des_pos_sub_.shutdown();
	};

	void StateEstControl::PosCallback(const geometry_msgs::PoseArray::ConstPtr& input_pos_array) {
		geometry_msgs::PoseArray pos_array = *input_pos_array;

		geometry_msgs::Pose last_pose;
		for(int i=0; i < pos_array.poses.size(); i++) {
			last_pose = pos_array.poses[i];
			pos_diff.x = last_pose.position.x;
			pos_diff.y = last_pose.position.y;
		}
		
	}

	void StateEstControl::IdCallback(const std_msgs::Int32MultiArray::ConstPtr& input_id_array) {
		std_msgs::Int32MultiArray id_array = *input_id_array;

		int32_t last_id = -1;
		for(int i=0; i < id_array.data.size(); i++) {
			if(id_array.data[i] != NULL)
				last_id = id_array.data[i];
		}

		if(last_id != -1) {
			id = last_id;
			updatePose();
		}
	}

	void StateEstControl::YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw) {
		std_msgs::Float32MultiArray yaw_array = *input_yaw;

		for(int i=0; i < yaw_array.data.size(); i++) {
			if(yaw_array.data[i] != NULL)
				pos.yaw = yaw_array.data[i];
		}

	}

	void StateEstControl::DesPosCallback(const geometry_msgs::PoseStamped::ConstPtr& input_des_pos) {
		des_pos = {
			*input_des_pos.pose.position.x,
			*input_des_pos.pose.position.y,
			*input_des_pos.pose.position.z
		};
		GoToDesPose();
	}

	void StateEstControl::updatePose() {
		pos.x = tag_array[id].x - pos_diff.x;
		pos.y = tag_array[id].y - pos_diff.y;

		geometry_msgs::PoseStamped state_est_pose;
		state_est_pose.pose.position.x = pos.x;
		state_est_pose.pose.position.y = pos.y;
		est_pos_pub_.publish(state_est_pose);
	}

	void StateEstControl::GoToDesPose() {
		while(1) {
			float dx = des_pos.x - pos.x;
			float dy = des_pos.y - pos.y;
			float dth = pos.yaw;

			updateAlpha(dx, dy, dth);
			updateBeta(dx, dy, dth);
			updateP(dx, dy, dth);

			float kp = 3;
			float ka = 8;
			float kb = -1.5;
			float v = kp * p;
			float w = ka * alpha + kb * beta;
			setWheel(v, w);
		}
	}

	void StateEstControl::updateAlpha(float dx, float dy, float dth) {
		alpha = - dth + atan2(dy, dx);
	}

	void StateEstControl::updateBeta(float dx, float dy, float dth) {
		beta = - dth - alpha;
	}

	void StateEstControl::updateP(float dx, float dy, float dth) {
		p = sqrt(dx*dx + dy*dy);
	}

	void StateEstControl::setWheel(float v, float w) {
		float L = CAR_WIDTH;
		float R = WHEEL_RADIUS;

		std_msgs::Float32 vr, vl;
		vr.data = (2*v + w*L) / (2*R);
		vl.data = (2*v - w*L) / (2*R);

		left_motor_pub_.publish(vl);
		right_motor_pub_.publish(vr);

	}

}