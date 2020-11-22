#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <std_srvs/SetBool.h>
#include <cstdlib>

int main(int argc, char **argv){
    ros::init(argc,argv, "emergency_stop_client");
    if(argc != 2){
        ROS_INFO("Incorrect usage!");
        return 1;
    }

    ros::NodeHandle nn;

    ros::ServiceClient client=nn.serviceClient<std_srvs::SetBool>("emergency_stop");
    std_srvs::SetBool service;

    if(atoi(argv[1]) == 1){
        ROS_INFO("---> True");
        service.request.data = true;
    } else{
        service.request.data = false;
    }

    if(client.call(service)){
        ROS_DEBUG_STREAM(service.response.message);
    } else{
        ROS_ERROR("Emergency_stop service call failed!");
        return 1;
    }
    return 0;
}