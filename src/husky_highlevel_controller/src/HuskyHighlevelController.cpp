#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  nodeHandle.getParam("/husky_highlevel_controller/laser_scan_topic_name", laser_scan_topic_name);
  nodeHandle.getParam("/husky_highlevel_controller/laser_scan_queue_size", laser_scan_queue_size);
  nodeHandle.getParam("/husky_highlevel_controller/cmd_vel_topic_name", cmd_vel_topic_name);
  nodeHandle.getParam("/husky_highlevel_controller/cmd_vel_queue_size", cmd_vel_queue_size);
  nodeHandle.getParam("/husky_highlevel_controller/visualization_topic_name", visualization_topic_name);
  nodeHandle.getParam("/husky_highlevel_controller/visualization_queue_size", visualization_queue_size);
  nodeHandle.getParam("/husky_highlevel_controller/zPosPillar", zPosPillar);
  nodeHandle.getParam("/husky_highlevel_controller/start_stop_topic_name", start_stop_topic_name);
  nodeHandle.getParam("/husky_highlevel_controller/start_stop_queue_size", start_stop_queue_size);

  
	laser_scan_subs_ = nodeHandle_.subscribe(laser_scan_topic_name, laser_scan_queue_size, &HuskyHighlevelController::laser_scan_Callback, this); 
  controlled_cmd_vel_publ_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name,cmd_vel_queue_size);
  pillar_vis_publ_ = nodeHandle_.advertise<visualization_msgs::Marker>(visualization_topic_name,visualization_queue_size);
  start_stop_subs_ = nodeHandle_.subscribe(start_stop_topic_name, start_stop_queue_size, &HuskyHighlevelController::start_stop_Callback, this);

  ROS_INFO_STREAM("Node launched");
}
void HuskyHighlevelController::laser_scan_Callback(const sensor_msgs::LaserScan &laser_scan_msgs){
  minDistance = 30;

  //calculate smallest distance and respective angle
  laser_scan_distance=laser_scan_msgs.ranges;
  for (int i = 0; i < laser_scan_distance.size(); i++){
    if (minDistance > laser_scan_distance[i]){
      minDistance = laser_scan_distance[i];
      pillarAngle = (laser_scan_msgs.angle_min + i * laser_scan_msgs.angle_increment);
    }
  }

  //pillar position
  xPosPillar = minDistance * cos(pillarAngle);
  yPosPillar = minDistance * sin(pillarAngle);

  //P-controller that drives husky towards the pillar
  //gains: linear and angular velocity
  float p_gain_vel = 0.1;
  float p_gain_ang = 0.4;

  if(xPosPillar > 0.4){
    cmd_vel_command.linear.x = xPosPillar * p_gain_vel;
    cmd_vel_command.angular.z = (-1) * yPosPillar * p_gain_ang;
  } else{
    cmd_vel_command.linear.x =  1.0;
    cmd_vel_command.angular.z = 0;
  }

  controlled_cmd_vel_publ_.publish(cmd_vel_command);

  //to visualize the pillar position
  pillar_vis_marker_func();

  //outputs
  ROS_INFO_STREAM("X position of pillar: " << xPosPillar);
  ROS_INFO_STREAM("Y position of pillar: " << yPosPillar);
  ROS_INFO_STREAM("ang position of pillar: " << pillarAngle);

  ROS_INFO_STREAM("Linear vel = " << cmd_vel_command.linear.x);
  ROS_INFO_STREAM("Angular vel = " << cmd_vel_command.angular.z);

}

void HuskyHighlevelController::start_stop_Callback(const std_msgs::Bool trigger){
  if(trigger.data==true){
    status_ = true;
  } else{
    status_ = false;
  }
}

void HuskyHighlevelController::pillar_vis_marker_func(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_laser";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.y = xPosPillar;
	marker.pose.position.x = yPosPillar;
	marker.pose.position.z = zPosPillar;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	pillar_vis_publ_.publish( marker );
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}


} /* namespace */
