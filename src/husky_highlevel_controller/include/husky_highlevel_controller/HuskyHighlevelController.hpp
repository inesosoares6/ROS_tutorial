#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


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

	//function related to pillar visualization
	//(http://wiki.ros.org/rviz/DisplayTypes/Marker)
	void pillar_vis_marker_func();

	vector<float> laser_scan_distance;

	//variables od config.yaml
	string laser_scan_topic_name;
	string cmd_vel_topic_name;
	string visualization_topic_name;
	int laser_scan_queue_size;
	int cmd_vel_queue_size;
	int visualization_queue_size;
	float zPosPillar;

	//smalest distance from laser scan
	float minDistance;

	// Pillar position
	float xPosPillar;
	float yPosPillar;
	float pillarAngle;

	//msg Twist
	geometry_msgs::Twist cmd_vel_command;

	//Subscribers
	ros::Subscriber laser_scan_subs;

	//Publishers
	ros::Publisher controlled_cmd_vel_publ;
	ros::Publisher pillar_vis_publ;
};

} /* namespace */
