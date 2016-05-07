#include <path_planner/path_planner.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace path_planner {
	class PathPlannerNodelet : public nodelet::Nodelet {
	public:
		PathPlannerNodelet(){}

	private:
		void onInit() {
			detector_.reset(new PathPlanner(getNodeHandle(), getPrivateNodeHandle()));
		}
		boost::shared_ptr<PathPlanner> detector_;
	};
}

PLUGINLIB_DECLARE_CLASS(path_planner, PathPlannerNodelet, path_planner::PathPlannerNodelet, nodelet::Nodelet);