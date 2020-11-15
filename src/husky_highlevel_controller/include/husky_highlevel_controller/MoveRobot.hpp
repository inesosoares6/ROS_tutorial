#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

using std::string;

namespace husky_highlevel_controller{

    class MoveRobot{

        public:
            MoveRobot();
            virtual ~MoveRobot();
            void moveCallback(const sensor_msgs::LaserScan::ConstPtr& moveRobot);

        private:
            ros::NodeHandle node;
            ros::Publisher moveRobot_pub;
            ros::Subscriber moveRobot_sub;
    };

}