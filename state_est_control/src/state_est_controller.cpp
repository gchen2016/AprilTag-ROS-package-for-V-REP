#include <state_est_control/state_est_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <XmlRpcException.h>

const int FLOOR_LENGTH = 5;

namespace state_est_control {
	StateEstControl::StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh) : pos_diff(Point(0, 0)), pos(Point(0, 0)) {

		// init tag array
		const int HALF_LENGTH = FLOOR_LENGTH / 2;
		for(int y=0; y < FLOOR_LENGTH; y++) {
			for(int x=0; x < FLOOR_LENGTH; x++)
				tag_array.push_back(Point(x - HALF_LENGTH, y - HALF_LENGTH));
		} 
			

		// subscriptions
		pos_sub_ = nh.subscribe("tag_detections_pose", 1, &StateEstControl::PosCallback, this);
		id_sub_ = nh.subscribe("tag_detections_id", 1, &StateEstControl::IdCallback, this);

		// publications
		est_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("state_est_pos", 1);
	}

	StateEstControl::~StateEstControl() {
		pos_sub_.shutdown();
		id_sub_.shutdown();
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

			pos.x = tag_array[id].x - pos_diff.x;
			pos.y = tag_array[id].y - pos_diff.y;

			geometry_msgs::PoseStamped state_est_pose;
			state_est_pose.pose.position.x = pos.x;
			state_est_pose.pose.position.y = pos.y;
			est_pos_pub_.publish(state_est_pose);
		}
	}

}