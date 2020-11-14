#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  nodeHandle.getParam("/husky_highlevel_controller/husky/topic_name", topic_name);
  nodeHandle.getParam("/husky_highlevel_controller/husky/queue_size", queue_size);
  nodeHandle.getParam("/husky_highlevel_controller/husky/control_gain", control_gain);
  ROS_INFO_STREAM(topic_name << " " << queue_size);
	laser_scan_subs = nodeHandle.subscribe(topic_name, queue_size, &HuskyHighlevelController::laser_scan_Callback, this); 
  controlled_cmd_vel_publ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel",1);
}
void HuskyHighlevelController::laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msgs){
  minDistance = 30;
  direction_index = 0;
  laser_scan_distance=laser_scan_msgs->ranges;
  ROS_INFO_STREAM("msgs received " << std::endl);
  for (int i = 0; i < laser_scan_distance.size(); i++){
    if (minDistance > laser_scan_distance[i]){
      minDistance = laser_scan_distance[i];
      direction_index = i;
    }
  }

  pillarAngle = -0.785 + 1.5 * pi/720 * direction_index;

  if(status_ == true){
    husky_angle_controller(2,pillarAngle);
  } else{
    husky_angle_controller(0,pi/2);
  }

  controlled_cmd_vel_publ.publish(cmd_vel_command);

  ROS_INFO_STREAM("The " << direction_index << "st message is chosen in " << laser_scan_distance.size() << " messages");
  ROS_INFO_STREAM("minDistance detected by laser: " << minDistance << ", angle is " << pillarAngle);

}

void HuskyHighlevelController::husky_angle_controller(float speed, float angle){
  cmd_vel_command.linear.x = speed;
  cmd_vel_command.angular.z = control_gain * (0-(angle-pi/2));
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}


} /* namespace */
