#include <state_est_control/state_est_controller.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace state_est_control {
	class StateEstContrllerNodelet : public nodelet::Nodelet {
	public:
		StateEstContrllerNodelet(){}

	private:
		void onInit() {
			detector_.reset(new StateEstControl(getNodeHandle(), getPrivateNodeHandle()));
		}
		boost::shared_ptr<StateEstControl> detector_;
	};
}

PLUGINLIB_DECLARE_CLASS(state_est_control, StateEstContrllerNodelet, state_est_control::StateEstContrllerNodelet, nodelet::Nodelet);