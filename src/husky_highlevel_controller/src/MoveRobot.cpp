#include "husky_highlevel_controller/MoveRobot.hpp"

namespace husky_highlevel_controller{

    MoveRobot::MoveRobot(){
        moveRobot_pub = node.advertise<sensor_msgs::LaserScan>("/moveRobot",1);
        moveRobot_sub = node.subscribe<sensor_msgs::LaserScan>("/scan",1,&MoveRobot::moveCallback, this);
    }

    void MoveRobot::moveCallback(const sensor_msgs::LaserScan::ConstPtr& moveRobot){
        int ranges = moveRobot->ranges.size();

        sensor_msgs::LaserScan scan;
        scan.header.stamp = moveRobot->header.stamp;
        scan.header.frame_id = moveRobot->header.frame_id;
        scan.angle_min = moveRobot->angle_min;
        scan.angle_max = moveRobot->angle_max;
        scan.angle_increment = moveRobot->angle_increment;
        scan.time_increment = moveRobot->time_increment;
        scan.range_min = 0;
        scan.range_max = 100;

        scan.ranges.resize(ranges);
        for(int i=0; i<ranges; i++){
            if(moveRobot->ranges[i]>30){
                scan.ranges[i] = 30;
            } else{
                scan.ranges[i]=moveRobot->ranges[i];
            }
            ROS_INFO_STREAM("/MoveRobot laser values:" << scan.ranges[i]);
        }

        ROS_INFO_STREAM("Maximum laser distance: " << scan.range_max);

        moveRobot_pub.publish(scan);
    }

    MoveRobot::~MoveRobot(){}

} /* namespace */