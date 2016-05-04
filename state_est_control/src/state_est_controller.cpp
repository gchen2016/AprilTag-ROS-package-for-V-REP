#include <state_est_control/state_est_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <XmlRpcException.h>

namespace state_est_control {
	StateEstControl::StateEstControl(ros::NodeHandle& nh, ros::NodeHandle& pnh) : pos_diff(Point(0, 0)), pos(Point(0, 0)) {

		// init tag array
		tag_array.push_back(Point(0, 0));
		tag_array.push_back(Point(1, 0));
		tag_array.push_back(Point(2, 0));
		tag_array.push_back(Point(3, 0));

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
		printf("In PosCallback\n");
		geometry_msgs::PoseArray pos_array = *input_pos_array;

		geometry_msgs::Pose last_pose;
		for(int i=0; i < (sizeof(pos_array)/sizeof(pos_array[0])); i++) {
			if(pos_array[i])
				&last_pose = pos_array[i];
		}

		if(!last_pose) {
			printf("Empty Pose\n");
			return;
		} else {
			printf("Pose not Empty\n");
			pos_diff.x = last_pose.position.x;
			pos_diff.y = last_pose.position.y;
		}
	}

	void StateEstControl::IdCallback(const std_msgs::Int32MultiArray::ConstPtr& input_id_array) {
		std_msgs::Int32MultiArray id_array = *input_id_array;

		int32_t last_id;
		for(int i=0; i < (sizeof(id_array)/sizeof(id_array[0])); i++) {
			if(id_array[i])
				last_id = id_array[i];
		}

		if(!last_id) {
			printf("Empty Id\n");
			return;
		} else {
			printf("Id not Empty\n");
			id = last_id;

			pos.x = tag_array[id].x + pos_diff.x;
			pos.y = tag_array[id].y + pos_diff.y;

			geometry_msgs::PoseStamped state_est_pose;
			state_est_pose.pose.position.x = pos.x;
			state_est_pose.pose.position.y = pos.y;
			est_pos_pub_.publish(state_est_pose);
		}
	}

}