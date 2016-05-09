#include <state_est_control/state_est_controller.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "state_est_control");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	state_est_control::StateEstControl detector(nh, pnh);
	ros::spin();
}