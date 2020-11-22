#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <std_srvs/SetBool.h>

ros::Publisher start_stop_publ_;

bool stop_(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response){
    if(request.data == false){
        start_stop_publ_.publish(false);
        response.success=false;
        response.message= "False";
    }
    if(request.data == true){
        start_stop_publ_.publish(true);
        response.success=true;
        response.message="True";
    }
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "emergency_stop_server");
    ros::NodeHandle nn;
    start_stop_publ_ = nn.advertise<std_msgs::Bool>("/start_stop",1);
    ros::ServiceServer service = nn.advertiseService("emergency_stop", stop_);
    ros::spin();
    return 0;
}