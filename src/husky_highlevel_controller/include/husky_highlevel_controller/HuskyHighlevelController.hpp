#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

using std::vector;
using std::string;

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msgs);
	vector<float> laser_scan_distance;
	string topic_name;
	int queue_size;
	float minDistance;
	int direction_index;

	ros::Subscriber laser_scan_subs;
};

} /* namespace */
