#include <state_est_control/state_est_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <XmlRpcException.h>

#include <math.h>

const int FLOOR_LENGTH = 5;

namespace state_est_control {
	const float CAR_WIDTH = 0.1;
	const float WHEEL_RADIUS = 0.04;

	StateEstControl::StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh) : pos_diff(Point(0, 0, 0)), pos(Point(0, 0, 0)), des_pos(Point(0, 0, 0)), isRandom(true) {

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
		yaw_sub_ = nh.subscribe("tag_detections_yaw", 1, &StateEstControl::YawCallback, this);

		// publications
		est_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("state_est_pos", 1);
		left_motor_pub_ = nh.advertise<std_msgs::Float64>("vrep/left_motor", 1);
		right_motor_pub_ = nh.advertise<std_msgs::Float64>("vrep/right_motor", 1);
		// left_motor_pub_ = nh.advertise<std_msgs::Float32>("left_motor", 1);
		// right_motor_pub_ = nh.advertise<std_msgs::Float32>("right_motor", 1);
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
		geometry_msgs::PoseStamped des_array = *input_des_pos;
		
		des_pos.x = des_array.pose.position.x;
		des_pos.y = des_array.pose.position.y;
		des_pos.yaw = des_array.pose.position.z;

		ROS_INFO("Des pos at x: %f, y: %f, yaw: %f", des_pos.x, des_pos.y, des_pos.yaw);

		isRandom = false;
		// GoToDesPose();
	}

	void StateEstControl::updatePose() {
		pos.x = tag_array[id].x - pos_diff.x;
		pos.y = tag_array[id].y - pos_diff.y;

		geometry_msgs::PoseStamped state_est_pose;
		state_est_pose.pose.position.x = pos.x;
		state_est_pose.pose.position.y = pos.y;
		est_pos_pub_.publish(state_est_pose);

		isRandom = false;

		GoToDesPose();
	}

	void StateEstControl::GoToDesPose() {
		if(isRandom)
			return;

		float dx = des_pos.x - pos.x;
		float dy = des_pos.y - pos.y;
		ROS_INFO("dx: %f, dy: %f\n", dx, dy);

		float yaw_diff = getYawDiff(pos.yaw, des_pos.yaw);
		ROS_INFO("yaw diff: %f, pos.yaw: %f, des_pos.yaw: %f", yaw_diff, pos.yaw, des_pos.yaw);

		float dth = pos.yaw;

		float v, w;

		// line move
		if(abs(yaw_diff) < 1) {
			updateAlpha(dx, dy, dth);
			updateBeta(dx, dy, dth);
			updateP(dx, dy, dth);

			float kp = 3;
			float ka = 8;
			float kb = -1.5;
			v = kp * p;
			w = ka * alpha + kb * beta;
			printf("alpha: %f, beta: %f\n", alpha, beta);
			printf("In yaw diff < 1\n");
		} else {
			v = 0;
			w = -0.1 * yaw_diff;
			printf("In yaw diff > 1\n");
		}

		setWheel(v, w);

		ROS_INFO("Set v: %f, w: %f", v, w);
		// isRandom = true;
	}

	void StateEstControl::updateAlpha(float dx, float dy, float dth) {
		alpha = - dth + atan2(dy, dx);
	}

	void StateEstControl::updateBeta(float dx, float dy, float dth) {
		beta = - alpha - dth + des_pos.yaw;
	}

	void StateEstControl::updateP(float dx, float dy, float dth) {
		p = sqrt(dx*dx + dy*dy);
	}

	void StateEstControl::setWheel(float v, float w) {
		v *= 0.1;
		float L = CAR_WIDTH;
		float R = WHEEL_RADIUS;

		std_msgs::Float64 vr, vl;
		vr.data = (2*v + w*L) / (2*R);
		vl.data = (2*v - w*L) / (2*R);

		left_motor_pub_.publish(vl);
		right_motor_pub_.publish(vr);

		ROS_INFO("left motor: %f, right motor: %f", vl.data, vr.data);

	}

	float StateEstControl::getYawDiff(float y1, float y2) {
		float diff = y1 - y2;
		if(diff < -3.1415926)
			diff += 3.1415926;
		else if(diff > 3.1415926)
			diff -= 3.1415926;

		return diff;
	}
}