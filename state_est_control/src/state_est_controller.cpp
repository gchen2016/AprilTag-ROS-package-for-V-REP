#include <state_est_control/state_est_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <XmlRpcException.h>

#include <iostream>
#include <math.h>

const int FLOOR_LENGTH = 5;
const float MOTOR_SCALAR = 1.3;

namespace state_est_control {
	const float CAR_WIDTH = 0.1655*2;
	const float WHEEL_RADIUS = 0.0975;
	const float PUB_RATE = 0.02;

	StateEstControl::StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh) : pos_diff(Point(0, 0, 0)), pos(Point(0, 0, 0)), des_pos(Point(0, 0, 0)), isRandom(true), isAprilTag(false) {

		// init tag array
		for(int y=0; y < FLOOR_LENGTH; y++) {
			for(int x=0; x < FLOOR_LENGTH; x++)
				tag_array.push_back(Point(x, y, 0));
		} 

		// init rotation matrix
		world_to_tag << -1, 0, 0,
		0, -1, 0,
		0, 0, 1;
		cam_to_robot << 1, 0, 0,
		0, -1, 0,
		0, 0, -1;

		// subscriptions
		pos_sub_ = nh.subscribe("tag_detections_pose", 1, &StateEstControl::PosCallback, this);
		id_sub_ = nh.subscribe("tag_detections_id", 1, &StateEstControl::IdCallback, this);
		des_pos_sub_ = nh.subscribe("des_pos", 1, &StateEstControl::DesPosCallback, this);
		yaw_sub_ = nh.subscribe("tag_detections_yaw", 1, &StateEstControl::YawCallback, this);
		left_state_sub_ = nh.subscribe("vrep/left_jointstate", 1, &StateEstControl::LeftMotorCallback, this);
		right_state_sub_ = nh.subscribe("vrep/right_jointstate", 1, &StateEstControl::RightMotorCallback, this);
		vrep_pos_sub_ = nh.subscribe("vrep/rob_position", 1, &StateEstControl::VrepPosCallback, this);

		// publications
		est_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("state_est_pos", 1);
		left_motor_pub_ = nh.advertise<std_msgs::Float64>("vrep/left_motor", 1);
		right_motor_pub_ = nh.advertise<std_msgs::Float64>("vrep/right_motor", 1);
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

		april_pos = last_pose;
		
	}

	void StateEstControl::IdCallback(const std_msgs::Int32MultiArray::ConstPtr& input_id_array) {
		std_msgs::Int32MultiArray id_array = *input_id_array;

		int32_t last_id = -1;
		for(int i=0; i < id_array.data.size(); i++) {
			if(id_array.data[i] > -1 && id_array.data[i] < 100) {
				last_id = id_array.data[i];
			}
		}

		if(last_id != -1) {
			id = last_id;
			isAprilTag = true;
			// UpdatePose(tag_array[id].x + pos_diff.x, tag_array[id].y + pos_diff.y, pos.yaw);
			PoseTransform();
			return;
		}

		isAprilTag = false;
	}

	void StateEstControl::YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw) {
		std_msgs::Float32MultiArray yaw_array = *input_yaw;

		for(int i=0; i < yaw_array.data.size(); i++) {
			if(yaw_array.data[i] < 4 || yaw_array.data[i] > -4)
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

	void StateEstControl::LeftMotorCallback(const sensor_msgs::JointState::ConstPtr& input_left_state) {
		sensor_msgs::JointState left_state_array = *input_left_state;
		left_v = - left_state_array.velocity[0] * MOTOR_SCALAR;

		if(!isAprilTag)
			AccumPose();
	}

	void StateEstControl::RightMotorCallback(const sensor_msgs::JointState::ConstPtr& input_right_state) {
		sensor_msgs::JointState right_state_array = *input_right_state;
		right_v = - right_state_array.velocity[0] * MOTOR_SCALAR;

		if(!isAprilTag)
			AccumPose();
	}

	void StateEstControl::VrepPosCallback(const geometry_msgs::PoseStamped::ConstPtr& input_vrep_pos) {
		geometry_msgs::PoseStamped vrep_pos_array = *input_vrep_pos;

		Point vrep_point = Point(
			vrep_pos_array.pose.position.x,
			vrep_pos_array.pose.position.y,
			0
			);

		ROS_INFO("Vrep position x: %f, y: %f\n", vrep_point.x, vrep_point.y);
	}

	void StateEstControl::PoseTransform() {
		UpdateTagMaxtrix();

		Eigen::Vector3d diff_cam(pos_diff.x, pos_diff.y, 0);
		Eigen::Vector3d diff_vector;

		diff_vector = - robot_to_world.inverse() * diff_cam;

		ROS_INFO("diff_vector: %f, %f, %f\n", diff_vector[0], diff_vector[1], diff_vector[2]);
		UpdatePose(tag_array[id].x + pos_diff.x, tag_array[id].y + pos_diff.y, pos.yaw);
	}

	void StateEstControl::UpdateTagMaxtrix() {
		float qw = april_pos.orientation.w;
		float qx = april_pos.orientation.x;
		float qy = april_pos.orientation.y;
		float qz = april_pos.orientation.z;

		tag_to_cam << 1 - 2*qy*qy - 2*qz*qz,
		2*qx*qy - 2*qz*qw,
		2*qx*qz + 2*qy*qw, 
		2*qx*qy + 2*qz*qw,
		1 - 2*qx*qx - 2*qz*qz,
		2*qy*qz - 2*qx*qw,
		2*qx*qz - 2*qy*qw,
		2*qy*qz + 2*qx*qw,
		1 - 2*qx*qx - 2*qy*qy;

		UpdateRobotMatrix();

	}

	void StateEstControl::UpdateRobotMatrix() {
		robot_to_world = cam_to_robot * tag_to_cam.inverse();
	}

	void StateEstControl::AccumPose() {
		float object_v = (left_v + right_v) * WHEEL_RADIUS / 2;
		float object_x = object_v * PUB_RATE;

		float accum_x = object_x * cos(pos.yaw);
		float accum_y = object_x * sin(pos.yaw);

		float object_w = (right_v - left_v) * WHEEL_RADIUS / CAR_WIDTH;
		float object_yaw = object_w * PUB_RATE;

		ROS_INFO("accum_x: %f, accum_y: %f\n", accum_x, accum_y);
		UpdatePose(pos.x + accum_x, pos.y + accum_y, normOmega(pos.yaw + object_yaw));
	}

	void StateEstControl::UpdatePose(float updateX, float updateY, float updateYaw) {
		pos.x = updateX;
		pos.y = updateY;
		pos.yaw = updateYaw;

		geometry_msgs::PoseStamped state_est_pose;
		state_est_pose.pose.position.x = pos.x;
		state_est_pose.pose.position.y = pos.y;
		est_pos_pub_.publish(state_est_pose);

		ROS_INFO("Current Pos x: %f, y: %f, yaw: %f", pos.x, pos.y, pos.yaw);

		GoToDesPose();
	}

	void StateEstControl::GoToDesPose() {
		if(isRandom)
			return;

		float dx = des_pos.x - pos.x;
		float dy = des_pos.y - pos.y;

		float yaw_diff = getYawDiff(pos.yaw, des_pos.yaw);
		ROS_INFO("yaw diff: %f, pos.yaw: %f, des_pos.yaw: %f", yaw_diff, pos.yaw, des_pos.yaw);

		float dth = pos.yaw;

		float v, w;

		if(fabs(dx) > 0.02 || fabs(dy) > 0.02) {
			updateAlpha(dx, dy, dth);
			updateBeta(dx, dy, dth);
			updateP(dx, dy, dth);

			ROS_INFO("Line move, dx: %f, dy: %f, yaw_diff: %f\n", dx, dy, yaw_diff);

			float kp = 3;
			float ka = 8;
			float kb = -1.5;
			v = kp * p;
			w = ka * alpha + kb * beta;
		} else {
			ROS_INFO("Rotation, dx: %f, dy: %f, yaw_diff: %f\n", dx, dy, yaw_diff);

			v = 0;
			w = -0.1 * yaw_diff;
		} 

		w = normOmega(w);
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
		v *= 0.15;
		float L = CAR_WIDTH;
		float R = WHEEL_RADIUS;

		std_msgs::Float64 vr, vl;
		vr.data = (2*v + w*L) / (2*R);
		vl.data = (2*v - w*L) / (2*R);

		left_motor_pub_.publish(vl);
		right_motor_pub_.publish(vr);

	}

	float StateEstControl::getYawDiff(float y1, float y2) {
		float diff = y1 - y2;
		// if(diff < -3.141592)
		// 	diff += 2*3.141592;
		// else if(diff > 3.141592)
		// 	diff -= 2*3.141592;

		diff = atan2(sin(diff), cos(diff));

		return diff;
	}

	float StateEstControl::normOmega(float input_w) {
		if(fabs(input_w) < 3.1416)
			return input_w;

		int q;
		if(input_w > 0) {
			q = (int)floor(input_w / 3.141592);
			return input_w - q * 3.141592;
		} else {
			q = (int)floor(- input_w / 3.141592);
			return input_w + q * 3.141592;
		}
	}
}